from Marking import Marking
from PyQt5.QtCore import Qt
from PyQt5.QtGui import QPixmap,QImage
from cv2 import  imread,circle,line,putText,cvtColor,COLOR_RGB2BGR
from os.path import exists 
from os import remove,listdir
from cv2 import  imread,circle,line,putText,cvtColor,COLOR_RGB2BGR
from math import atan,pi
from xml.etree.ElementTree import Element,ElementTree
import xml.etree.ElementTree as ET
class UmbrellaCone(Marking):
    """description of class"""
    def __init__(self,classlabel):
        self.classlabel=classlabel
        super(UmbrellaCone, self).__init__()
        self.tips()
    def tips(self):
        self.printmessage("**************************************************说明**************************************************",time=False)
        self.printmessage("*****提示*****：请确保打开路径中不含中文，否则图片可能打不开！！！文件格式支持jpg、png",time=False)
        self.printmessage("**操作快捷键**：Q打开文件；A上一张；D下一张；S保存；R重置当前图片；鼠标左击选点、右击撤销上一步",time=False)
        self.printmessage("***操作说明***：图片框鼠标左键依次单击目标对象上下左右四点和中心点，保存自动进入下一张",time=False)
    def calculate(self,point):
        xmin=min(point[0][0],point[1][0],point[2][0],point[3][0])
        ymin=min(point[0][1],point[1][1],point[2][1],point[3][1])
        xmax=max(point[0][0],point[1][0],point[2][0],point[3][0])
        ymax=max(point[0][1],point[1][1],point[2][1],point[3][1])
        return [(xmin,ymin),(xmax,ymax),(point[4][0],point[4][1])]
    def savexml(self,path,f,delscr):
        result=[]
        if len(self.vertex)==5:
            if delscr:
                remove(path)
            result=self.calculate(self.vertex)
            annotation = Element('annotation')
            tree = ElementTree(annotation)
            folder= Element('folder')
            folder.text=f
            annotation.append(folder)

            filename= Element('filename')
            filename.text=self.IMGfile[self.currentimg]
            annotation.append(filename)

            Path= Element('path')
            Path.text=path.replace("/","\\")
            annotation.append(Path)

            Size= Element('size')
            width= Element('width')
            width.text=str(self.Draw[-1].shape[1])
            Size.append(width)

            height= Element('height')
            height.text=str(self.Draw[-1].shape[0])
            Size.append(height)

            depth= Element('depth')
            depth.text=str(self.Draw[-1].shape[2])
            Size.append(depth)
            annotation.append(Size)

            object= Element('object')
            name= Element('name')
            name.text=self.classlabel
            object.append(name)

            bndbox= Element('bndbox')
            
            xmin=Element('xmin')
            xmin.text=str(result[0][0])
            bndbox.append(xmin)
            ymin=Element('ymin')
            ymin.text=str(result[0][1])
            bndbox.append(ymin)

            xmax=Element('xmax')
            xmax.text=str(result[1][0])
            bndbox.append(xmax)
            ymax=Element('ymax')
            ymax.text=str(result[1][1])
            bndbox.append(ymax)
            object.append(bndbox)

            center=Element('center')
            x=Element('x')
            y=Element('y')
            x.text,y.text=str(result[2][0]),str(result[2][1])
            center.append(x)
            center.append(y)
            object.append(center)
            annotation.append(object)
            self.indent(annotation)
            tree.write(path, xml_declaration=True)
            return True
        else:
            return False
            
    def MouseRightPressEvent(self,x,y):
        if len(self.IMGfile)==0:
            return
        if len(self.IMG_TEMP)>1 :
            self.IMG_TEMP.pop()
            self.vertex.pop()
            img1=self.IMG_TEMP[-1]
            frame = cvtColor(img1, COLOR_RGB2BGR)
            img = QImage(frame.data, frame.shape[1], frame.shape[0],frame.shape[1]*3, QImage.Format_RGB888)#第四个参数设置通道数对齐,不然图片可能会变形
            self.label.setPixmap(QPixmap.fromImage(img))
            self.label.setScaledContents(True)#图片大小与label适应

            message="回撤到第"+str(len(self.vertex))+"个点。"
            self.statusbar.showMessage(message,5000)
    def MouseLeftPressEvent(self,X,Y):
        if len(self.IMGfile)==0:
            return
        if  len(self.vertex)<5 :
            img1=self.IMG_TEMP[-1].copy()
            x=img1.shape[1]*X/self.label.width()
            y=img1.shape[0]*Y/self.label.height()
            l=max(int((img1.shape[1]/self.label.width()+img1.shape[0]/self.label.height())/2),1)
            img_=circle(img1,center =(int(x),int(y)),radius = 10*l,color = (0,0,255),thickness = -1)
            self.IMG_TEMP.append(img_)
            frame = cvtColor(img_, COLOR_RGB2BGR)
            img = QImage(frame.data, frame.shape[1], frame.shape[0],frame.shape[1]*3, QImage.Format_RGB888)#第四个参数设置通道数对齐,不然图片可能会变形
            self.label.setPixmap(QPixmap.fromImage(img))
            self.label.setScaledContents(True)#图片大小与label适应
            self.vertex.append((int(x),int(y)))
            if len(self.vertex)==5:
                self.IMG_TEMP.pop()
                drawdata=self.calculate(self.vertex)
                iimg=self.Draw[-1].copy()
                self.draw(iimg,drawdata)
                self.IMG_TEMP.append(iimg)
                frame = cvtColor(iimg, COLOR_RGB2BGR)
                img = QImage(frame.data, frame.shape[1], frame.shape[0],frame.shape[1]*3, QImage.Format_RGB888)#第四个参数设置通道数对齐,不然图片可能会变形
                self.label.setPixmap(QPixmap.fromImage(img))
                self.label.setScaledContents(True)#图片大小与label适应
                message="完成"
                self.statusbar.showMessage(message,5000)
            else:
                message="请继续点击第"+str(len(self.vertex)+1)+"个点。"
                self.statusbar.showMessage(message,5000)
    def draw(self,img,data):
        l=max(int((img.shape[1]/self.label.width()+img.shape[0]/self.label.height())/2),1)
        line(img,(data[0][0],data[0][1]),(data[0][0],data[1][1]),color=(255,0,0),thickness=4*l)
        line(img,(data[0][0],data[0][1]),(data[1][0],data[0][1]),color=(255,0,0),thickness=4*l)
        line(img,(data[1][0],data[1][1]),(data[1][0],data[0][1]),color=(255,0,0),thickness=4*l)
        line(img,(data[1][0],data[1][1]),(data[0][0],data[1][1]),color=(255,0,0),thickness=4*l)
        img=circle(img,center =(data[2][0],data[2][1]),radius = 10*l,color = (0,0,255),thickness = -1)
    def loadXML(self,xml_path,img):
        tree = ET.parse(xml_path)
        root = tree.getroot()
        for obj in root.iter('object'):
            if obj.find('name').text !=self.classlabel:
                return
            for bndbox in obj.iter('bndbox'):
                xmin = int(bndbox.find('xmin').text)
                ymin = int(bndbox.find('ymin').text)
                xmax = int(bndbox.find('xmax').text)
                ymax = int(bndbox.find('ymax').text)
            for center in obj.iter('center'):
                cx = int(center.find('x').text)
                cy = int(center.find('y').text)
            self.draw(img,[[xmin,ymin],[xmax,ymax],[cx,cy]])