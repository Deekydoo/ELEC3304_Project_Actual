from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import FancyBboxPatch
out=Path(__file__).parent/'figures_new'/'plant_diagram'
out.mkdir(parents=True,exist_ok=True)
plt.rcParams.update({'font.family':'DejaVu Sans','svg.fonttype':'none'})
fig,ax=plt.subplots(figsize=(18,10)); ax.set(xlim=(0,20),ylim=(0,11)); ax.axis('off')
fig.patch.set_facecolor('white')
ink='#233e55'; line='#526c81'
def box(x,y,w,h,title,body='',color='#edf3f9'):
 ax.add_patch(FancyBboxPatch((x,y),w,h,boxstyle='round,pad=0.02,rounding_size=0.10',facecolor=color,edgecolor='#59758c',linewidth=1.2,zorder=2))
 ax.text(x+w/2,y+h-.30,title,ha='center',va='center',fontsize=11.5,fontweight='bold',color=ink,zorder=3)
 if body: ax.text(x+w/2,y+h*.34,body,ha='center',va='center',fontsize=10,linespacing=1.65,zorder=3)
def arrow(points,label=None,pos=None):
 xs,ys=zip(*points); ax.plot(xs,ys,color=line,lw=1.35,zorder=1)
 ax.annotate('',xy=points[-1],xytext=points[-2],arrowprops=dict(arrowstyle='->',color=line,lw=1.35),zorder=4)
 if label: ax.text(*pos,label,fontsize=10,color=ink,ha='center',va='center',bbox=dict(facecolor='white',edgecolor='none',pad=2),zorder=5)
def dot(x,y): ax.plot(x,y,'o',color=line,ms=4,zorder=5)
ax.text(.35,10.5,'Microduck Plant-Model',fontsize=25,fontweight='bold',color=ink)
ax.text(.35,9.98,'36 physical states  |  14 coordinates + 14 velocities + 8 motor torques',fontsize=12,color=line)
box(.35,7.3,2.3,1.6,'Joint commands',r'$u \in \mathbb{R}^{8}$'+'\nTarget joint angles')
box(3.1,7.3,2.2,1.6,'Command limits','Joint-angle\nsaturation')
box(5.8,7.3,2.8,1.6,'Internal servos',r'$\tau_{\rm req}=K_p(u-q_j)-K_d v_j$')
box(9.1,7.3,2.3,1.6,'Torque limits',r'$|\tau_{\rm target}|\leq 0.60\ {\rm N\,m}$')
box(11.9,7.3,2.4,1.6,'Actuator lag',r'$\dot{\tau}=(\tau_{\rm target}-\tau)/T_a$')
box(14.9,7.3,4.7,1.6,'Rigid-body dynamics',r'$\ddot q=M(q)^{-1}[S\tau+Q_c+Q_w$'+'\n'+r'$-h(q,v)-Q_d(v)]$')
for a,b in [(2.65,3.1),(5.3,5.8),(8.6,9.1),(11.4,11.9),(14.3,14.9)]: arrow([(a,8.05),(b,8.05)])
box(15.25,9.65,4.0,.85,'External torso push',r'Force $F$ and moment $m$','#fff3df')
arrow([(17.25,9.65),(17.25,8.9)],r'$Q_w=J_v^{T}F+J_\omega^{T}m$',(17.25,9.25))
box(14.9,4.4,4.7,1.65,'State integration',r'$\ddot q\ \longrightarrow\ v\ \longrightarrow\ q$'+'\nPosition and velocity states')
arrow([(17.25,7.3),(17.25,6.05)],r'$\ddot q$',(17.55,6.65))
box(9.1,4.4,4.2,1.65,'Foot-ground contact','8 sole contact points\nNormal spring/damper + friction','#edf6ef')
arrow([(14.9,5.2),(13.3,5.2)],r'$q,\ v$',(14.1,5.43))
arrow([(11.2,6.05),(11.2,6.65),(15.7,6.65),(15.7,7.3)],r'$Q_c$',(13.4,6.65))
box(.35,4.4,5.0,1.65,'Ideal sensor model','Gyroscope (3) + accelerometer (3)\nJoint encoders (8)','#edf6ef')
# One shared state/output bus keeps feedback paths aligned and explicit.
arrow([(18.65,4.4),(18.65,3.55),(2.85,3.55),(2.85,4.4)],r'$q,\ v,\ \ddot q$',(12.0,3.55))
arrow([(7.2,3.55),(7.2,7.3)],r'$q_j,\ v_j$'+ '\nLocal servo feedback',(7.2,6.25)); dot(7.2,3.55)
box(.35,1.95,5.0,1.35,'Sampled measurements','50 Hz; optional sensor bias + sine\n14 outputs to the observer','#fff3df')
arrow([(1.15,4.4),(1.15,3.3)])
ax.text(6.25,2.65,'Physical outputs: roll, pitch, joint motion and delivered motor torques',fontsize=11,color=ink)
ax.text(6.25,2.12,'Sensor errors are applied to measurements, outside the physical dynamics.',fontsize=10.5,color=line)
ax.plot([.35,19.6],[1.53,1.53],color='#cbd6df',lw=1)
ax.text(.35,1.15,'LINEAR REPRESENTATION',fontsize=11,fontweight='bold',color=ink)
ax.text(4.45,1.15,r'$\delta\dot x=A\delta x+B\delta u+E\delta w\qquad \delta y=C\delta x+F_w\delta w$',fontsize=14,color=ink)
ax.text(.35,.61,'Linearized about the standing equilibrium; command and torque saturation are bypassed in the linear plant.',fontsize=10.5,color=line)
ax.text(.35,.16,r'Nominal servos: $K_p=0.55$, $K_d=0$, $T_a=0.03\ {\rm s}$.  Outer state feedback and observer are outside this plant diagram.',fontsize=10.5,color=line)
fig.subplots_adjust(left=.015,right=.99,top=.98,bottom=.03)
for ext in ['png','svg','pdf']: fig.savefig(out/f'microduck_plant_block_diagram.{ext}',dpi=200,bbox_inches='tight',facecolor='white')
print(out/'microduck_plant_block_diagram.png')

