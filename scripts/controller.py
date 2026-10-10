#!/usr/bin/env python3
"""Start/stop only this extracted project's owned controller process."""
import argparse,datetime,json,os,re,signal,subprocess,time
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
p=argparse.ArgumentParser();p.add_argument('operation',choices=['start','stop']);p.add_argument('kind',nargs='?',choices=['standing','walking'],default='walking');args=p.parse_args()
state=ROOT/'runtime/current_session.json';exe=ROOT/'build/controller/wb_nmpc_controller'
if args.operation=='stop':
 rec=json.loads(state.read_text());pid=rec['pid'];cmd=Path(f'/proc/{pid}/cmdline')
 if cmd.exists():
  assert str(exe).encode() in cmd.read_bytes().split(b'\0'),'PID no longer belongs to this controller; nothing stopped'
  os.killpg(pid,signal.SIGTERM)
 print('Stop requested for owned controller',pid);raise SystemExit
assert exe.is_file(),'Run bash scripts/build.sh first'
assert (ROOT/'install/local_setup.bash').is_file(),'Local ROS2 dependency build is required'
if state.exists():
 rec=json.loads(state.read_text());cmd=Path(f"/proc/{rec['pid']}/cmdline")
 assert not (cmd.exists() and str(exe).encode() in cmd.read_bytes().split(b'\0')),'An owned controller is already running; stop it first'
mem=int(re.search(r'MemAvailable:\s+(\d+)',Path('/proc/meminfo').read_text()).group(1))/2**20
assert mem>=3.5,f'Need >=3.5 GiB available before runner/Simscape startup; currently {mem:.2f}'
duration=4.8 if args.kind=='walking' else 1.0
folder=ROOT/'runtime'/f"{args.kind}_{datetime.datetime.now().strftime('%Y%m%d_%H%M%S_%f')}";folder.mkdir(parents=True)
(folder/'cppad_code_gen').mkdir()
env=os.environ.copy()
for key in list(env):
 if key.startswith(('TASK3','TASK4','TASK5','G3E_')) or key in ['LD_LIBRARY_PATH','LD_PRELOAD','PYTHONPATH','PYTHONHOME']:env.pop(key)
env.update(json.loads((ROOT/'config/controller_environment.json').read_text()))
for key,name in [('TASK5C_GEOMETRY_CACHE','moment_ad'),('TASK5C_COLLISION_CACHE','collision_ad'),('TASK5C_FOOT_COST_CACHE','foot_cost_ad')]:
 cache=ROOT/'build'/name;cache.mkdir(parents=True,exist_ok=True);env[key]=str(cache)
env.update(TASK5A_AUTHORITY='SIMULINK_REFERENCE',TASK5A_PACKET_FILE=str(folder/'reference_packet.bin'),ROS_DOMAIN_ID='31',ROS_LOG_DIR=str(folder/'ros_logs'))
argv=[str(exe),'simscape',str(ROOT/'config/task.info'),str(ROOT/'config/reference.info'),str(ROOT/'assets/g1_29dof.urdf'),str(ROOT/'config/gait.info'),'unused',str(duration),str(folder),'0','0','0','unused']
cmd=['bash','-c','source /opt/ros/jazzy/setup.bash; source "$1/install/local_setup.bash"; export LD_LIBRARY_PATH="$1/build/controller:$LD_LIBRARY_PATH"; shift; exec "$@"','g1-walking',str(ROOT),*argv]
with (folder/'controller.log').open('wb') as log:
 proc=subprocess.Popen(cmd,cwd=folder,env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
state.write_text(json.dumps(dict(pid=proc.pid,folder=str(folder),duration=duration,standing=args.kind=='standing',exe=str(exe)),indent=2)+'\n')
try:
 deadline=time.monotonic()+600
 while time.monotonic()<deadline:
  assert proc.poll() is None,f'Controller exited; inspect {folder}/controller.log'
  if 'WAITING_FOR_VALID_PLANT_STATE' in (folder/'controller.log').read_text(errors='replace'):
   print('READY: run run_walking in MATLAB. Output:',folder);break
  time.sleep(.25)
 else:raise TimeoutError('Controller startup exceeded 600 s; source-only first launch may compile AD libraries')
except BaseException:
 if proc.poll() is None:os.killpg(proc.pid,signal.SIGTERM)
 raise
