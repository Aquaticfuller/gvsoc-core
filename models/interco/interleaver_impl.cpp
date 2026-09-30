/*
 * Copyright (C) 2020 GreenWaves Technologies, SAS, ETH Zurich and
 *                    University of Bologna
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/*
 * Authors: Germain Haugou, GreenWaves Technologies (germain.haugou@greenwaves-technologies.com)
 */

#include <vp/vp.hpp>
#include <vp/itf/io.hpp>
#include <stdio.h>
#include <math.h>
#include <algorithm>
#include <deque>
#include <unordered_map>
#include <vector>

class interleaver : public vp::Component
{

public:

    interleaver(vp::ComponentConf &conf);

  static vp::IoReqStatus req(vp::Block *__this, vp::IoReq *req);


  static void grant(vp::Block *__this, vp::IoReq *req);

  static void response(vp::Block *__this, vp::IoReq *req);

private:
  // A downstream target may retain a request until its grant/response.
  // Each transfer owns a distinct child request, reused only after a
  // stripe completes, and keeps loader zero-fill data alive.
  struct Transfer
  {
    vp::IoReq *parent;
    vp::IoReq child;
    uint64_t addr;
    uint64_t size;
    uint64_t done = 0;
    uint64_t chunk_size = 0;
    uint8_t *data;
    std::vector<uint8_t> write_data;
  };

  static void async_handler(vp::Block *__this, vp::ClockEvent *event);
  void async_enqueue(Transfer *transfer);
  void async_complete(Transfer *transfer);
  bool asynchronous;
  vp::ClockEvent *async_event;
  std::deque<Transfer *> ready;
  std::unordered_map<vp::IoReq *, Transfer *> transfers;

  vp::Trace     trace;

  vp::IoMaster **out;
  vp::IoSlave **masters_in;
  vp::IoSlave in;

  int nb_slaves;
  int nb_masters;
  int interleaving_bits;
  int stage_bits;
  int enable_shift;
  uint64_t offset_mask;
  uint64_t remove_offset;
  bool offset_translation;
};

interleaver::interleaver(vp::ComponentConf &config)
: vp::Component(config)
{
  traces.new_trace("trace", &trace, vp::DEBUG);

  in.set_req_meth(&interleaver::req);
  new_slave_port("input", &in);

  nb_slaves = get_js_config()->get_child_int("nb_slaves");
  nb_masters = get_js_config()->get_child_int("nb_masters");
  stage_bits = get_js_config()->get_child_int("stage_bits");
  interleaving_bits = get_js_config()->get_child_int("interleaving_bits");
  remove_offset = get_js_config()->get_child_int("remove_offset");
  enable_shift = get_js_config()->get_child_int("enable_shift");
  offset_translation = get_js_config()->get_child_bool("offset_translation");
  asynchronous = get_js_config()->get_child_bool("asynchronous");
  async_event = asynchronous ? event_new(interleaver::async_handler) : nullptr;

  if (stage_bits == 0)
  {
    stage_bits = log2(nb_slaves);
  }

  offset_mask = -1;
  offset_mask &= ~((1 << (interleaving_bits + stage_bits)) - 1);

  out = new vp::IoMaster *[nb_slaves];
  for (int i=0; i<nb_slaves; i++)
  {
    out[i] = new vp::IoMaster();
    out[i]->set_resp_meth(&interleaver::response);
    out[i]->set_grant_meth(&interleaver::grant);
    new_master_port("out_" + std::to_string(i), out[i]);
  }

  masters_in = new vp::IoSlave *[nb_masters];
  for (int i=0; i<nb_masters; i++)
  {
    masters_in[i] = new vp::IoSlave();
    masters_in[i]->set_req_meth(&interleaver::req);
    new_slave_port("in_" + std::to_string(i), masters_in[i]);
  }

}

vp::IoReqStatus interleaver::req(vp::Block *__this, vp::IoReq *req)
{
  interleaver *_this = (interleaver *)__this;
  uint64_t offset = req->get_addr();
  bool is_write = req->get_is_write();
  uint64_t size = req->get_size();
  uint8_t *data = req->get_data();
  int64_t latency = req->get_latency();

  if (_this->asynchronous)
  {
    auto *transfer = new Transfer();
    transfer->parent = req;
    transfer->addr = offset - _this->remove_offset;
    transfer->size = size;
    transfer->data = data;
    if (is_write && data)
    {
      transfer->write_data.assign(data, data + size);
      transfer->data = transfer->write_data.data();
    }
    _this->transfers.emplace(&transfer->child, transfer);
    _this->async_enqueue(transfer);
    return vp::IO_REQ_PENDING;
  }

  uint8_t *init_data = data;
  uint64_t init_size = size;
  uint64_t init_offset = offset;
  int64_t init_latency = latency;
  int64_t max_latency = 0;

  _this->trace.msg("Received IO req (offset: 0x%llx, size: 0x%llx, is_write: %d, latency: %ld)\n", offset, size, is_write, latency);

  int port_size = 1<<_this->interleaving_bits;
  int align_size = offset & (port_size - 1);
  if (align_size) align_size = port_size - align_size;

  offset -= _this->remove_offset;

  while(size) {

    int loop_size = port_size;
    if (align_size) {
      loop_size = align_size;
      align_size = 0;
    }
    if (loop_size > size) loop_size = size;

    int output_id = (offset >> _this->interleaving_bits) & ((1 << _this->stage_bits) - 1);
    uint64_t new_offset;
    if (_this->offset_translation)
    {
        new_offset = ((offset & _this->offset_mask) >> _this->stage_bits) + (offset & ((1<<_this->interleaving_bits)-1));
    }
    else if (_this->enable_shift)
    {
      new_offset = ((offset >> _this->enable_shift) & (-1ULL << _this->interleaving_bits)) | (offset & ((1ULL << _this->interleaving_bits) - 1));
    }
    else
    {
      new_offset = offset;
    }

    _this->trace.msg("Forwarding interleaved packet (port: %d, offset: 0x%x, size: 0x%x)\n", output_id, new_offset, loop_size);

    if (!_this->out[output_id]) return vp::IO_REQ_INVALID;

    req->set_addr(new_offset);
    req->set_size(loop_size);
    req->set_data(data);
    req->set_exact_latency(init_latency);

    vp::IoReqStatus err = _this->out[output_id]->req_forward(req);
    if (err != vp::IO_REQ_OK)
    {
      // Temporary hack, until this component supports asynchronous requests.
      // If we get an asynchronous reply and we are done, just return
      if (err == vp::IO_REQ_PENDING && size == loop_size)
      {
        return vp::IO_REQ_PENDING;
      }
      else if(err == vp::IO_REQ_DENIED)
      {
        // Propagate DENIED (not INVALID) even mid-split: the caller retries the whole request;
        // the chunks already forwarded are rewritten with identical data (writes are idempotent).
        // Without this a busy DRAMSys channel turns a big loader write into a fatal INVALID.
        return vp::IO_REQ_DENIED;
      }
      else
      {
        return vp::IO_REQ_INVALID;
      }
    }

    int64_t iter_latency = req->get_latency();
    if (iter_latency > max_latency)
    {
      max_latency = iter_latency;
    }

    size -= loop_size;
    offset += loop_size;
    if (data)
      data += loop_size;
  }

  req->set_addr(init_offset);
  req->set_size(init_size);
  req->set_data(init_data);
  req->set_latency(max_latency);


  return vp::IO_REQ_OK;
}

void interleaver::grant(vp::Block *__this, vp::IoReq *req)
{
  // Asynchronous targets retain DENIED requests and later respond to
  // them. Do not reissue on grant or notify the PENDING upstream parent.
}

void interleaver::response(vp::Block *__this, vp::IoReq *req)
{
  interleaver *_this = (interleaver *)__this;
  if (_this->asynchronous)
  {
    auto *transfer = _this->transfers.at(req);
    transfer->done += transfer->chunk_size;
    _this->async_enqueue(transfer);
  }
}

void interleaver::async_enqueue(Transfer *transfer)
{
  ready.push_back(transfer);
  if (!async_event->is_enqueued()) event_enqueue(async_event, 1);
}

void interleaver::async_complete(Transfer *transfer)
{
  vp::IoReq *parent = transfer->parent;
  transfers.erase(&transfer->child);
  delete transfer;
  parent->get_resp_port()->resp(parent);
}

void interleaver::async_handler(vp::Block *__this, vp::ClockEvent *event)
{
  interleaver *_this = (interleaver *)__this;
  // Work on independent transfers in parallel; a denied channel retains
  // only its own child, so other channels can continue accepting traffic.
  size_t count = _this->ready.size();
  while (count--)
  {
    auto *transfer = _this->ready.front();
    _this->ready.pop_front();
    while (transfer->done < transfer->size)
    {
      uint64_t addr = transfer->addr + transfer->done;
      uint64_t stripe = 1ULL << _this->interleaving_bits;
      int output_id = (addr >> _this->interleaving_bits) & ((1 << _this->stage_bits) - 1);
      uint64_t offset = addr;
      if (_this->offset_translation)
        offset = ((addr & _this->offset_mask) >> _this->stage_bits) + (addr & (stripe - 1));
      else if (_this->enable_shift)
        offset = ((addr >> _this->enable_shift) & (-1ULL << _this->interleaving_bits)) | (addr & (stripe - 1));

      transfer->chunk_size = std::min(stripe - (addr & (stripe - 1)), transfer->size - transfer->done);
      transfer->child.init();
      transfer->child.set_addr(offset);
      transfer->child.set_size(transfer->chunk_size);
      transfer->child.set_data(transfer->data ? transfer->data + transfer->done : nullptr);
      transfer->child.set_is_write(transfer->parent->get_is_write());
      vp::IoReqStatus status = _this->out[output_id]->req(&transfer->child);
      if (status == vp::IO_REQ_PENDING || status == vp::IO_REQ_DENIED) break;
      if (status != vp::IO_REQ_OK)
        _this->trace.fatal("Invalid asynchronous interleaver access at 0x%llx\n", addr);
      transfer->done += transfer->chunk_size;
    }
    if (transfer->done == transfer->size) _this->async_complete(transfer);
  }
}

extern "C" vp::Component *gv_new(vp::ComponentConf &config)
{
  return new interleaver(config);
}
