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

ax.text(.35,10.5,'Microduck Full Control Model',fontsize=25,fontweight='bold',color=ink)
ax.text(.35,10.0,'Observer-based state feedback  |  Standing balance regulation  |  Sampling period: 20 ms (50 Hz)',fontsize=12,color=line)
box(.35,7.1,3.1,1.7,'State feedback',r'$\Delta u_{\rm req}[k]=-K\hat z[k]$'+'\nPole-placement gain')
box(4.0,7.1,2.8,1.7,'Correction limits',r'$\Delta u=\mathrm{sat}_{\pm 8^\circ}$'+'\nEight joint offsets')
box(7.35,7.1,2.8,1.7,'Standing targets',r'$u_{\rm req}=u_{\rm eq}+\Delta u$')
box(10.7,7.1,3.0,1.7,'Joint limits + hold','Absolute command limits\nZero-order hold: 20 ms')
box(14.35,6.7,5.2,2.1,'Microduck plant','Inner servos + torque limits + actuator lag\nRigid-body dynamics + foot contact\n36 physical states; 8 command inputs')
for a,b in [(3.45,4),(6.8,7.35),(10.15,10.7),(13.7,14.35)]: arrow([(a,7.9),(b,7.9)])
box(15.1,9.35,3.7,.85,'External push','Forward / sideways force','#fff3df')
arrow([(16.95,9.35),(16.95,8.8)],r'$w(t)$',(17.45,9.05))
box(14.35,3.9,5.2,1.5,'Sensor model + sampling','Gyro (3), accelerometer (3), encoders (8)\nSample ideal measurements at 50 Hz','#edf6ef')
arrow([(16.95,6.7),(16.95,5.4)],r'$q,\ v,\ \ddot q$',(17.65,6.05))
box(14.35,1.55,5.2,1.35,'Measurement errors',r'$y_m[k]=y[k]+b+a\sin(\omega kT_s)$'+'\nOptional constant bias and sinusoidal error','#fff3df')
arrow([(16.95,3.9),(16.95,2.9)])
box(9.65,1.55,3.7,1.35,'Sensor normalization',r'$y_n=D_y^{-1}(y_m-y_{\rm eq})$')
arrow([(14.35,2.2),(13.35,2.2)],r'$y_m$',(13.85,2.45))
box(.35,1.55,8.1,3.4,'Discrete Luenberger observer',r'$\hat z[k+1]=A_d\hat z[k]+B_d\Delta u_{\rm applied}[k]$'+'\n'+r'$\qquad +\ L\left(y_n[k]-C_n\hat z[k]\right)$'+'\n\n33 normalized retained states; pole-placement gain L\nOne-sample memory stores the state estimate','#edf6ef')
arrow([(9.65,2.2),(8.45,2.2)],r'$y_n$',(9.05,2.45))
# Known limited command is used by both the physical plant and predictor.
arrow([(14.0,7.9),(14.0,5.8),(9.0,5.8),(9.0,4.2),(8.45,4.2)],r'$\Delta u_{\rm applied}=u_{\rm applied}-u_{\rm eq}$',(11.4,5.8)); dot(14,7.9)
arrow([(1.9,4.95),(1.9,7.1)],r'$\hat z[k]$',(2.4,6.0))
ax.text(4.0,6.25,'Zero roll/pitch deviation target\nabout the standing equilibrium',fontsize=10.5,color=line,ha='left',linespacing=1.7)
ax.plot([.35,19.55],[1.05,1.05],color='#cbd6df',lw=1)
ax.text(.35,.65,r'$z=S^{-1}(x_r-x_{r,\rm eq})$;  retained states exclude absolute world X, Y and yaw.  $D_y$ scales the sensor channels.',fontsize=11,color=ink)
ax.text(.35,.15,'Nonlinear plant: command/torque limits active. Linear comparison: limits bypassed; same observer and state-feedback gains.',fontsize=10.5,color=line)
fig.subplots_adjust(left=.015,right=.99,top=.98,bottom=.03)
for ext in ['png','svg','pdf']: fig.savefig(out/f'microduck_full_control_block_diagram.{ext}',dpi=200,bbox_inches='tight',facecolor='white')
print(out/'microduck_full_control_block_diagram.png')
