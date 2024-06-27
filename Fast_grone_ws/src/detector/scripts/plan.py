import cv2 as cv
from geometry_msgs.msg import PoseStamped
from quadrotor_msgs.msg import TakeoffLand
import serial
import serial.tools.list_ports
from ultralytics import YOLO
from detector.srv import NumberNet_data, QRCode_data, YoloNet_data
import rospy
import cv2

def wait_for_arrived():
    rospy.set_param("EGO_Arrived", False)
    while rospy.get_param("EGO_Arrived") is False:
        pass
    rospy.sleep(rospy.Duration(3))
    print("等待到达目的地")

def sent_point(x,y,z):
    point_msg.pose.position.x = x
    point_msg.pose.position.y = y
    point_msg.pose.position.z = z
    waypointPub.publish(point_msg)
    print("已发布节点：", 'x:', point_msg.pose.position.x, 'y:', point_msg.pose.position.y, 'z:', point_msg.pose.position.z)

def Identify_picture():
    class_id=""
    ret,src = cap.read()
    img = src.copy()
    cv.imwrite("img1234.png",src)
    gray=cv.cvtColor(img,cv.COLOR_BGR2GRAY)

    retval, dst = cv.threshold( gray, 220, 255, cv.THRESH_BINARY_INV ) 

    con,his = cv.findContours(dst,cv.RETR_CCOMP,cv.CHAIN_APPROX_SIMPLE)

    x,y,w,h=0,0,4,4
    for c in con:
        x1,y1,w1,h1 = cv.boundingRect(c)
        if 10000<(w1*h1)<40000 and (w1*h1)>(w*h):
                x=x1
                y=y1
                w=w1
                h=h1
    roi = src[ int(y+h/4):int(y+3*h/4),int(x+w/4):int(x+3*w/4) ]
   
    results = model(roi)  # predict on an image
    for r in results:
            i=0
            while i<3:
                try:
                    cls=r.probs.top1
                    with open("/home/kexie/Desktop/python/data/cifar-100.yaml","r") as fd:
                        class_id=fd.readlines()[cls+8]
                        class_id=class_id.split(" ")
                        class_id=class_id[3][:-1] 
                        # print("class="+class_id) 
                        # print(class_id) 
                        # print(type(class_id))
                        break
                except:
                    i+=1
                    continue   

    return class_id
def throw_cuba(x,y,z):
    z-=0.7
    sent_point(x,y,z)
    wait_for_arrived()
    wait_for_arrived()
    cuba=square.pop(0)
    write=ser.write(f"{cuba} 90/n".encode('utf-8'))
    print(f"throw {cuba}")
    rospy.sleep(rospy.Duration(2))
    z+=0.7
    sent_point(x,y,z)
    wait_for_arrived()
    wait_for_arrived()




rospy.init_node('detect_node')
point_msg = PoseStamped()
waypointPub = rospy.Publisher("move_goal", PoseStamped, queue_size=10)
landPub = rospy.Publisher("px4ctrl/takeoff_land", TakeoffLand, queue_size=10)

ser = serial.Serial("/dev/ttyUSB1",9600)
if (ser.isOpen()):
    print("open serial success")
square=["C","B","A"]

cap=cv.VideoCapture(6)
print("open cap")

model = YOLO("/home/kexie/Fast_drone_ws/src/detector/scripts/yolov8n-cls.pt")  # load an official model
model = YOLO("/home/kexie/Fast_drone_ws/src/detector/scripts/2024 6 21 19.pt")  # load a custom model


x=1.8
y=0
z=1.2
#二维码
wait_for_arrived()
sent_point(x,y,z)
wait_for_arrived()
wait_for_arrived()
now_point=0

class_id_1=""
class_id_2=""
direction=""

# 识别二维码
i=True
while i:
    ret,img=cap.read()
    # cv2.imshow("img",img)
    # cv2.waitKey(10)
    gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
    # 检测
    qrcoder = cv.QRCodeDetector()
    # 解码
    codeinfo, points, straight_qrcode = qrcoder.detectAndDecode(gray)
    if codeinfo!="":
        i=False
        print("qrcode : %s" % codeinfo)
        with open("/home/kexie/Desktop/QRcoder.txt","w") as fd:
            fd.write(codeinfo)

        codeinfo=codeinfo.split(",")
        class_id_1=str(codeinfo[0])
        class_id_2=str(codeinfo[1])
        direction=str(codeinfo[2])
        i=False

rospy.sleep(rospy.Duration(3))

#img1
y+=1.6 
sent_point(x,y,z)
wait_for_arrived()
wait_for_arrived()
now_point+=1


#img2
x+=1.8
sent_point(x,y,z)
wait_for_arrived()
wait_for_arrived()
now_point+=1
throw_cuba(x,y,z)

# #特殊
# x = 6
# y = 1
# z = 1.2
# sent_point(x,y,z)
# wait_for_arrived()
# wait_for_arrived()
# now_point+=1
# throw_cuba(x,y,z)

# #img3
# x = 3.6
# y = 1.6
# z = 1.2
# sent_point(x,y,z)
# wait_for_arrived()
# wait_for_arrived()
# now_point+=1
# # throw_cuba()


#img3
x = 3.6
y = -1.6
z = 1.2
sent_point(x,y,z)
wait_for_arrived()
wait_for_arrived()
now_point+=1
#throe
throw_cuba(x,y,z)

#img4
x = 1.8
y = -1.6
z = 1.2
sent_point(x,y,z)
wait_for_arrived()
wait_for_arrived()
now_point+=1
#thorw
throw_cuba(x,y,z)

#img4
x = 0
y = -1.6
z = 1.2
sent_point(x,y,z)
wait_for_arrived()
wait_for_arrived()
now_point+=1

if direction=="left":
    x = 0
    y = 1.6
    z = 1.2
    sent_point(x,y,z)
    wait_for_arrived()
    wait_for_arrived()
    now_point+=1
else:pass

wait_for_arrived()
wait_for_arrived()
land_msg = TakeoffLand()
land_msg.takeoff_land_cmd = 2
landPub.publish(land_msg)
print("准备落地")