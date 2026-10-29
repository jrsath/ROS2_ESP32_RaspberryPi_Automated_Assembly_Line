# Editable report figures, using the same blue and green scheme as Assignment 3.
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import FancyBboxPatch, FancyArrowPatch
out=Path(__file__).parent

def figure(w,h):
 f,a=plt.subplots(figsize=(w,h));a.set_xlim(0,100);a.set_ylim(0,100);a.axis('off');return f,a

def box(a,x,y,w,h,title,body,colour='#eef5fc'):
 a.add_patch(FancyBboxPatch((x,y),w,h,boxstyle='round,pad=0.5,rounding_size=2',facecolor=colour,edgecolor='#91afc8',linewidth=1))
 a.text(x+w/2,y+h-5,title,ha='center',va='top',fontsize=10,weight='bold',color='#20394f')
 a.text(x+w/2,y+h/2-2,body,ha='center',va='center',fontsize=8.5,color='#20394f',linespacing=1.5)

def arrow(a,start,end,colour='#3377ac'):
 a.add_patch(FancyArrowPatch(start,end,arrowstyle='-|>',mutation_scale=12,lw=1.6,color=colour))

def save(f,name):
 f.savefig(out/(name+'.png'),dpi=210,bbox_inches='tight',facecolor='white')
 f.savefig(out/(name+'.svg'),bbox_inches='tight',facecolor='white');plt.close(f)

f,a=figure(10,5)
box(a,2,61,28,32,'ROS 2 coordinator','Assembly / belt / packaging\nCamera requests and results\nWaypoints + ID.yaml')
box(a,36,61,28,32,'micro-ROS agent','Simulation: desktop client\nPhysical: ESP32 USB / Wi-Fi\nCommands and feedback')
box(a,70,61,28,32,'Arm 3 camera','Live image + STL outline\nWaiting / Verifying / Pass / Fail\nNo-camera bypass is logged')
box(a,2,10,28,30,'Four SO101 arms','Gazebo controllers in simulation\nLab feedback: read-only currently\nMeasured joint feedback','#edf8f4')
box(a,36,10,28,30,'ESP32 / virtual board','Core 0: GPIO + step pulses\nCore 1: micro-ROS\nWatchdog and input debounce','#edf8f4')
box(a,70,10,28,30,'Cell equipment','Start / Pause / selector\nConveyor + proximity sensor\nTower lights + alarm','#edf8f4')
arrow(a,(30,77),(36,77));a.plot([84,84,16],[93,97,97],color='#3377ac',lw=1.6);arrow(a,(16,97),(16,93));arrow(a,(16,61),(16,40));arrow(a,(50,61),(50,40));arrow(a,(64,25),(70,25))
a.text(50,0,'Hardware E-stop cuts actuator/relay power independently of ROS.',ha='center',fontsize=9,color='#20394f')
save(f,'architecture')

f,a=figure(11,7)
a.text(50,99,'Project 2 | Board interface',ha='center',va='top',fontsize=16,weight='bold',color='#20394f')
box(a,34,18,32,69,'ESP32 Dev Module','USB to laptop / micro-ROS\n\nInputs: GPIO2, GPIO4, GPIO16\nProximity: GPIO35\n\nSTEP 18 / DIR 19 / ENABLE 21\nTower: 26 / 25 / 33 / 32\n\n3.3V logic only','#edf8f4')
box(a,1,65,27,23,'Control box','Red → GPIO2\nGreen → GPIO4\nSelector → GPIO16')
box(a,1,30,27,23,'Proximity sensor','SIGNAL → GPIO35\nSupply and GND per sensor\nExternal bias required')
box(a,72,65,27,23,'Stepper driver','PUL ← GPIO18\nDIR ← GPIO19\nEN ← GPIO21')
box(a,72,30,27,23,'Tower interface','G 26 / Y 25 / R 33\nAlarm 32\nUse rated 3.3V-to-relay interface')
arrow(a,(28,76),(34,76));arrow(a,(28,41),(34,41));arrow(a,(66,76),(72,76));arrow(a,(66,41),(72,41))
a.text(50,11,'NC E-stop: separate hardwired power-cut circuit. Feedback pin is not yet assigned.',ha='center',fontsize=9,color='#a34321')
a.text(50,5,'Pin labels transcribed from the supplied sketch. Connector order and input polarity require confirmation.',ha='center',fontsize=8.5,color='#20394f')
a.text(50,0,'Do not connect the tower relay supply or a 5V sensor output directly to an ESP32 GPIO.',ha='center',fontsize=9,color='#a34321')
f.savefig(out/'board_interface.pdf',bbox_inches='tight',facecolor='white');save(f,'board_interface')

f,a=figure(10,4)
steps=[('1 · Base and gear','Arm 2 places base rod-up\nArm 1 fits 2V4 onto the rod'),('2 · End insert','Arm 1 stages 3V4 and holds base\nArm 2 inserts through the end'),('3 · Inspect and pack','Arm 3 inspects and loads toy\nArm 4 lids and stacks box')]
for i,(title,body) in enumerate(steps):
 box(a,2+i*34,29,28,48,title,body,'#edf8f4' if i==1 else '#eef5fc')
 if i<2:arrow(a,(30+i*34,53),(36+i*34,53))
a.text(50,9,'Return clear before the other arm enters. Start the next build after belt transfer begins.',ha='center',fontsize=10,color='#20394f');save(f,'sequence')
