import ehci

#ehci.ehciInit()
print "before"
ehci.ehciInit()
ehci.ehciLoop(3,1)
print "after"


import direct.directbase.DirectStart
from pandac.PandaModules import *

from direct.task import Task
from direct.actor import Actor
from direct.interval.IntervalGlobal import *
import math



#Load the first environment model
environ = loader.loadModel("models/environment")
environ.reparentTo(render)
environ.setScale(0.25,0.25,0.25)
environ.setPos(-8,42,0)


#Task to move the camera
def SpinCameraTask(task,hist=[],yhist=[],frame=[]):
  initialGuess=0
  
  if(len(frame) < 10):
    frame.append(1)
    initialGuess=1

  print "Looping",initialGuess
  ehci.ehciLoop(3,initialGuess)
  print "Looping",initialGuess
  mat = ehci.getGlPositMatrix()
  print "After"
#  u,v,w,x = ehci.getHeadBounds()
  u = 50*mat[12]
  v = 50*mat[13]
  w = 50*mat[14]
  angle=math.atan2(-mat[2],mat[0])
  print "Ang ",angle
  if(u>0):
    angleradians = math.atan(480.0/u) #task.time * 6.0
  else:
    angleradians = 0
  angledegrees = angleradians * (180.0/ math.pi)
  hist.append(angledegrees)
  #calculate means because of noise
  if(len(hist)>10): hist.pop(0)
  yhist.append(v)
  if(len(yhist)>10): yhist.pop(0)
  #angle = sum(hist)/len(hist)
  angle = angle * (180.0/math.pi)
  ymean = sum(yhist)/len(yhist)
  base.camera.setPos(u/10.0,-w/10.0,-v/10.0+3)
#  base.camera.setPos(-20 - w/5,-10-ymean/10.0,+10)
  base.camera.setHpr(angle, 0, 0)
  print u,v,w


  #angledegrees = task.time * 6.0
  #angleradians = angledegrees * (math.pi / 180.0)
  #base.camera.setPos(20*math.sin(angleradians),-20.0*math.cos(angleradians),3)
  #base.camera.setHpr(angledegrees, 0, 0)
  return Task.cont

taskMgr.add(SpinCameraTask, "SpinCameraTask")

#Load the panda actor, and loop its animation
pandaActor = Actor.Actor("models/panda-model",{"walk":"models/panda-walk4"})
pandaActor.setScale(0.005,0.005,0.005)
pandaActor.reparentTo(render)
pandaActor.loop("walk")

#Create the four lerp intervals needed to walk back and forth
pandaPosInterval1= pandaActor.posInterval(13,Point3(0,-10,0), startPos=Point3(0,10,0))
pandaPosInterval2= pandaActor.posInterval(13,Point3(0,10,0), startPos=Point3(0,-10,0))
pandaHprInterval1= pandaActor.hprInterval(3,Point3(180,0,0), startHpr=Point3(0,0,0))
pandaHprInterval2= pandaActor.hprInterval(3,Point3(0,0,0), startHpr=Point3(180,0,0))

#Create and play the sequence that coordinates the intervals
pandaPace = Sequence(pandaPosInterval1, pandaHprInterval1,
  pandaPosInterval2, pandaHprInterval2, name = "pandaPace")
pandaPace.loop()

run()

