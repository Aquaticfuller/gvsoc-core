"""Compile the production spill-drain method against a deterministic IO endpoint."""
from pathlib import Path
import subprocess
import tempfile

source = Path(__file__).resolve().parents[2] / 'models/cpu/iss_v2/src/cores/spatz/spatz_vlsu_v2.cpp'
text = source.read_text()
start = text.index('void VuLsu::drain_narrow_spills()')
method = text[start:]
fixture = r'''
#include <cassert>
#include <cstdint>
#include <deque>
#include <vector>
namespace vp { struct IoReq {}; enum IoReqStatus { IO_REQ_DENIED, IO_REQ_DONE, IO_REQ_GRANTED }; }
struct Clock { int64_t cycle=0; int64_t get_cycles() { return cycle; } };
struct Iss { Clock clock; }; struct Vu { Iss iss; };
struct Port {
  bool deny=false; int calls=0;
  vp::IoReqStatus req(vp::IoReq*) { calls++; return deny ? vp::IO_REQ_DENIED : vp::IO_REQ_DONE; }
};
struct VuLsu {
  Vu vu; int nb_ports=2, completed=0;
  struct Spill { vp::IoReq *req; int64_t cycle; };
  std::vector<std::deque<Spill>> narrow_requests{2};
  std::vector<bool> port_stalled{false,false};
  std::vector<int64_t> narrow_service_cycle{-1,-1}, narrow_full_cycle{-1,-1};
  std::vector<vp::IoReq*> denied_reqs{nullptr,nullptr};
  std::vector<Port> ports{2};
  void handle_done(vp::IoReq*) { completed++; }
  void drain_narrow_spills();
};
'''
main = r'''
int main() {
 VuLsu m; vp::IoReq a,b,c;
 m.narrow_requests[0].push_back({&a,1});
 m.narrow_requests[0].push_back({&b,1});
 m.narrow_requests[1].push_back({&c,1});
 m.drain_narrow_spills(); assert(m.completed==0);
 m.vu.iss.clock.cycle=1; m.ports[0].deny=true;
 m.drain_narrow_spills(); assert(m.completed==1);
 assert(m.denied_reqs[0]==&a && m.narrow_requests[0].size()==2);
 m.drain_narrow_spills(); assert(m.ports[0].calls==1);
 m.port_stalled[0]=false; m.ports[0].deny=false;
 m.vu.iss.clock.cycle=2; m.drain_narrow_spills();
 assert(m.narrow_full_cycle[0]==2 && m.completed==2);
 m.drain_narrow_spills(); assert(m.completed==2);
 m.vu.iss.clock.cycle=3; m.drain_narrow_spills();
 assert(m.completed==3 && m.narrow_requests[0].empty());
}
'''
with tempfile.TemporaryDirectory(prefix='spatz-spill-test-') as tmp:
 path=Path(tmp); (path/'test.cpp').write_text(fixture+method+main)
 subprocess.run(['g++','-std=c++17','-Wall','-Wextra','-Werror',str(path/'test.cpp'),'-o',str(path/'test')],check=True)
 subprocess.run([str(path/'test')],check=True)
print('PASS registered issue, denied retention, independent ports, full-buffer edge and one service/port/cycle')
