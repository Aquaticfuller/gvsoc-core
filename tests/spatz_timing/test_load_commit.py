"""Exercise the production ROB drain against controlled response ordering."""
from pathlib import Path
import subprocess
import tempfile

source = Path(__file__).resolve().parents[2] / 'models/cpu/iss_v2/src/cores/spatz/spatz_vlsu_v2.cpp'
s = source.read_text()
method = s[s.index('void VuLsu::commit_narrow_loads()'):s.index('// H1 admission condition')]
fixture = r'''
#include <algorithm>
#include <cassert>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <deque>
#include <map>
#include <set>
#include <string>
#include <vector>
struct iss_insn_t {};
struct PendingInsn { int id=0, entry=0; };
struct Clock { int64_t cycle=0; int64_t get_cycles(){return cycle;} };
struct Exec { iss_insn_t insn; iss_insn_t *get_insn(int){return &insn;} };
struct Iss {Clock clock; Exec exec; std::string get_path(){return "fixture";} };
struct Vu { Iss iss; int committed=0; void exec_insn_chunk(iss_insn_t*,PendingInsn*,int,int,int){}
 void insn_commit(PendingInsn*,int n){committed+=n;} };
struct Trace { template<typename...T> void msg(const char*,T...){} };
struct Count {int n=1;int get(){return n;}};
struct VuLsuPendingInsn {PendingInsn *insn;int nb_remaining_bursts=4,nb_pending_bursts=4;};
std::map<const void*,long> vlsu_stall_streak,vlsu_last_retire;
std::set<const void*> vlsu_stall_reported;
struct VuLsu {
 static constexpr int queue_size=4;
 struct Req {int get_size(){return 4;}};
 struct VlsuReq {Req req;VuLsuPendingInsn *slot;int vstart=0,nb_elem=1;};
 struct VlsuRobEntry {bool allocated=true,valid=true;VlsuReq *req;int64_t ready_cycle=1;};
 struct NarrowCommit {int64_t cycle;std::vector<VlsuReq*> words;};
 Vu vu;Trace trace;bool clocked_narrow=true;int64_t narrow_commit_cycle=-1;
 std::deque<NarrowCommit> narrow_commits;
 Count nb_pending_insn;int insn_first=0,nb_ports=2,load_port_base=0;
 PendingInsn pending;std::vector<VuLsuPendingInsn> insns;
 std::vector<VlsuReq> reqs;std::vector<VlsuReq*> reqs_free;
 std::vector<std::vector<VlsuRobEntry>> rob;
 std::vector<int> rob_first{0,0},rob_count{2,2};std::vector<bool> port_stalled{false,false};
 VuLsu(){
   insns.resize(4);insns[0].insn=&pending;reqs.resize(4);rob.resize(2);
   for(int p=0;p<2;p++)for(int i=0;i<2;i++){
     reqs[p*2+i].slot=&insns[0];rob[p].push_back({true,true,&reqs[p*2+i],1});
   }
 }
 void commit_narrow_loads();
};
'''
main = r'''
int main(){
 VuLsu m;
 m.commit_narrow_loads();assert(m.narrow_commits.empty()); // not same-edge visible
 m.vu.iss.clock.cycle=1;m.rob[1][0].valid=false;
 m.commit_narrow_loads();assert(m.narrow_commits.empty()); // all required lanes
 m.rob[1][0].valid=true;m.commit_narrow_loads();
 assert(m.narrow_commits.size()==1 && m.narrow_commits.front().cycle==2);
 assert(m.rob_count[0]==1 && m.insns[0].nb_pending_bursts==4 && m.vu.committed==0);
 m.commit_narrow_loads();assert(m.narrow_commits.size()==1); // one row per edge
 m.vu.iss.clock.cycle=2;m.commit_narrow_loads();assert(m.narrow_commits.size()==2);

 // An arrived response from a younger load cannot complete an older row.
 VuLsu tagged;tagged.vu.iss.clock.cycle=1;
 tagged.reqs[2].slot=&tagged.insns[1];
 tagged.commit_narrow_loads();
 assert(tagged.narrow_commits.empty() && tagged.rob_count[0]==2 && tagged.rob_count[1]==2);
 tagged.reqs[2].slot=&tagged.insns[0];tagged.commit_narrow_loads();
 assert(tagged.narrow_commits.size()==1);

 // Three real words occupy a full row and a one-lane tail. The other
 // tail lane already holds a younger response; it supplies no padding data
 // and keeps its request and pending-beat count when the older load drains.
 VuLsu tail;tail.insns[0].nb_remaining_bursts=3;tail.insns[0].nb_pending_bursts=3;
 tail.insns[1].nb_pending_bursts=1;tail.reqs[3].slot=&tail.insns[1];
 tail.vu.iss.clock.cycle=1;tail.commit_narrow_loads();
 tail.vu.iss.clock.cycle=2;tail.commit_narrow_loads();
 assert(tail.narrow_commits.size()==2 && tail.narrow_commits.back().words.size()==1);
 assert(tail.narrow_commits.back().words.front()==&tail.reqs[1]);
 assert(tail.insns[0].nb_remaining_bursts==0 && tail.rob_count[0]==0);
 assert(tail.rob_count[1]==1 && tail.rob_first[1]==1 && tail.rob[1][1].allocated);
 assert(tail.insns[1].nb_pending_bursts==1 && tail.vu.committed==0);

 VuLsu old;old.clocked_narrow=false;old.commit_narrow_loads();
 assert(old.vu.committed==16 && old.insns[0].nb_pending_bursts==0);
}
'''
with tempfile.TemporaryDirectory(prefix='spatz-commit-test-') as tmp:
 p=Path(tmp);(p/'test.cpp').write_text(fixture+method+main)
 subprocess.run(['g++','-std=c++17','-Wall','-Wextra',str(p/'test.cpp'),'-o',str(p/'test')],check=True)
 subprocess.run([str(p/'test')],check=True)
print('PASS next-edge visibility, required-lane readiness, instruction tags, partial tails, one row/cycle, delayed writeback and legacy control')
