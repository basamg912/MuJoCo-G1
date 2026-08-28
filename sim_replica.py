"""Faithful python replica of the C++ MuJoCo deployment (observation.cpp + controller.cpp).
Runs policy.pt in the loop to reproduce/diagnose the standing instability."""
import mujoco, numpy as np, torch, sys

XML = "model/kapex/kapex_play.xml"
PT  = "pyfile/exported/policy.pt"

isaac_to_mj = np.array([0,7,14,1,8,15,2,9,16,3,10,17,24,4,11,18,25,5,12,19,26,6,13,20,27,21,28,22,29,23,30])

KP=np.array([83.83,83.83,277.15,277.15,40,40,5]*2+[228.204,456.408,456.408]+[83.827,83.827,8.01,8.01,2.842,5,5]*2)
KD=np.array([10.67,10.67,20,20,2,2,0.05]*2+[29.056,58.112,58.112]+[10.673,10.673,1.02,1.02,0.362,0.05,0.05]*2)
LIM=np.array([70,70,180,180,60,60,20]*2+[100,200,200]+[70,70,30,30,20,10,10]*2)

# ---- options controllable from CLI for A/B ----
ANGVEL_BODY_RAW = True   # True: raw qvel[3:6] (deployment). False: rotate (legacy _temp)
HIST_OLDEST_FIRST = True # True: oldest->newest (deployment). False: reversed
VEL_CMD = np.array([0.0,0.0,0.0])
PUSH_T = None; PUSH_V = 0.5
for a in sys.argv[1:]:
    if a=="angvel_rot": ANGVEL_BODY_RAW=False
    if a=="hist_rev": HIST_OLDEST_FIRST=False
    if a=="oldkd": KD[2]=KD[3]=KD[9]=KD[10]=35.3
    if a.startswith("push"):
        PUSH_T=2.0
        try: PUSH_V=float(a[4:])
        except: pass

m = mujoco.MjModel.from_xml_path(XML); d = mujoco.MjData(m)
if "stiff" in sys.argv:
    m.geom_solref[:,0]=0.005; m.geom_solref[:,1]=1.0   # stiffer/faster contact (timeconst=timestep)
    m.opt.cone = mujoco.mjtCone.mjCONE_ELLIPTIC; m.opt.impratio=100
kid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_KEY,"home")
mujoco.mj_resetDataKeyframe(m,d,kid); mujoco.mj_forward(m,d)
qhome = d.qpos[7:38].copy()
pel = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY,"pelvis")

policy = torch.jit.load(PT); policy.eval()
try: policy.reset()
except Exception: pass

def single_obs(last_action):
    bqi=np.zeros(4); mujoco.mju_negQuat(bqi, d.xquat[pel])
    if ANGVEL_BODY_RAW:
        av = d.qvel[3:6]*0.2
    else:
        tmp=np.zeros(3); mujoco.mju_rotVecQuat(tmp, d.qvel[3:6].copy(), bqi); av=tmp*0.2
    g=np.zeros(3); mujoco.mju_rotVecQuat(g, np.array([0,0,-1.0]), bqi)
    q=d.qpos[7:38]; qd=d.qvel[6:38]
    jpos=(q-qhome)[isaac_to_mj]
    jvel=(qd*0.05)[isaac_to_mj]
    return np.concatenate([av, g, VEL_CMD, jpos, jvel, last_action]).astype(np.float32)  # 102

SINGLE=102; H=5; ts=[0,3,6,9,40,71]; sz=[3,3,3,31,31,31]
hist=[]; last_action=np.zeros(31,dtype=np.float32)

def stack():
    out=np.zeros(SINGLE*H,dtype=np.float32); idx=0
    order=range(H) if HIST_OLDEST_FIRST else range(H-1,-1,-1)
    for s,z in zip(ts,sz):
        for j in order:
            out[idx:idx+z]=hist[j][s:s+z]; idx+=z
    return out

q_des = qhome.copy()
N=int(5.0/m.opt.timestep); step_count=0
maxtilt=0; minz=d.qpos[2]; log=[]
for k in range(N):
    if k%4==0:  # policy @ 50Hz
        so=single_obs(last_action)
        if not hist: hist=[so.copy() for _ in range(H)]
        else:
            hist.append(so.copy())
            if len(hist)>H: hist.pop(0)
        with torch.no_grad():
            a=policy(torch.from_numpy(stack()).unsqueeze(0)).squeeze(0).numpy()
        last_action=a.astype(np.float32)
        amj=np.zeros(31); amj[isaac_to_mj]=a
        q_des = qhome + 0.25*amj
    if PUSH_T is not None and abs(k*m.opt.timestep - PUSH_T) < m.opt.timestep/2:
        d.qvel[0] = PUSH_V; print(f"  >>> PUSH at t={k*m.opt.timestep:.2f}: set base vx={PUSH_V}")
    q=d.qpos[7:38]; qd=d.qvel[6:38]
    d.ctrl[:31]=np.clip(KP*(q_des-q)-KD*qd,-LIM,LIM)
    mujoco.mj_step(m,d)
    g=np.zeros(3); bqi=np.zeros(4); mujoco.mju_negQuat(bqi,d.xquat[pel]); mujoco.mju_rotVecQuat(g,np.array([0,0,-1.0]),bqi)
    tilt=np.degrees(np.arccos(np.clip(-g[2],-1,1))); maxtilt=max(maxtilt,tilt); minz=min(minz,d.qpos[2])
    if k%100==0: log.append((k*m.opt.timestep, d.qpos[2], tilt, float(np.abs(last_action).max())))

print(f"CONFIG angvel_raw={ANGVEL_BODY_RAW} hist_oldest_first={HIST_OLDEST_FIRST} cmd={VEL_CMD}")
print(f"{'t':>5} {'base_z':>7} {'tilt°':>6} {'max|a|':>7}")
for t,z,ti,a in log: print(f"{t:5.2f} {z:7.3f} {ti:6.1f} {a:7.2f}")
print(f"RESULT: min_z={minz:.3f} max_tilt={maxtilt:.1f}  -> {'FELL' if (minz<0.55 or maxtilt>40) else 'STAYED UP'}")
