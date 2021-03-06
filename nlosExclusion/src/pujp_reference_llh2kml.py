#! /usr/bin/env python
# -*- coding=utf-8 -*-
# finished by Dr. WEN
"""
    Function: read lat lon height from jingdong standard dataset
    Welson Wen, Ph.D.
    https://sites.google.com/view/weisongwen/about-me

    subcribe : '/ublox_gps_node/fix' latitude longitude

"""
import rospy
import rospy
from pykml.factory import KML_ElementMaker as KML 
import csv # csv reading needed library
from lxml import etree 

from sensor_msgs.msg   import NavSatFix # standard message type for GNSSs


class pullh2kml_eval():
    def __init__(self):
        rospy.Subscriber('/fix2', NavSatFix, self.callublox_llh)
        self.lat_ = [] # used to save latitude
        self.lon_ = [] # used to save longitude
        self.GPS_Week_Second = 0.0
        self.writeToKML = 0.0
        self.standardGT2kml()

    def callublox_llh(self,data):
        self.navsatfix_ = NavSatFix()
        self.navsatfix_ = data
        self.lat_.append(float(self.navsatfix_.latitude))
        print 'len(self.lat_)',len(self.lat_)
        self.lon_.append(float(self.navsatfix_.longitude))
        self.GPS_Week_Second = self.navsatfix_.header.stamp
        print 'GPS_Week_Second',self.GPS_Week_Second

    def standardGT2kml(self):
        print 'begin read standard data'
        self.Fcsv_GNSS = csv.reader(open('/home/wws/Downloads/referenceshinjuku.csv','r'))  # read csv context to csv_reader variable
        for rowCsv in self.Fcsv_GNSS:
            # print 'rowCsv.size () -> ',len(rowCsv)
            self.lat_.append(float(rowCsv[2]))
            self.lon_.append(float(rowCsv[3]))
            
        print 'finish tranversal standard data',len(self.lat_)




if __name__ == '__main__':
    rospy.init_node('pullh2kml_evaluublox', anonymous=True)
    pullh2kml_eval_ =pullh2kml_eval()
    rate = rospy.Rate(0.002)#
    preTim = 0.0
    while not rospy.is_shutdown():
        #rate.sleep()
        # print 'sleep end ',preTim,pullh2kml_eval_.GPS_Week_Second
        #if( (preTim > 0) and (preTim == pullh2kml_eval_.GPS_Week_Second) and (pullh2kml_eval_.writeToKML ==0)):
	if( len(pullh2kml_eval_.lon_)>5):
            # print 'finished...and begin to write llh to kml'
            pullh2kml_eval_.writeToKML = 1
            # ????????????????????????Folder
            fold = KML.Folder(KML.Placemark(
                KML.Point(KML.coordinates(str(pullh2kml_eval_.lon_[0]) + ',' + str(pullh2kml_eval_.lat_[0]) + ',0'))
            )
            )
            # ????????????????????????Folder???
            for i in range(1, len(pullh2kml_eval_.lon_)):
                fold.append(KML.Placemark(
                    KML.Point(KML.coordinates(str(pullh2kml_eval_.lon_[i]) + ',' + str(pullh2kml_eval_.lat_[i]) + ',0')))
                )
            # ??????etree???KML??????????????????????????????
            content = etree.tostring(etree.ElementTree(fold), pretty_print=True)
            # ????????????????????????????????????Google??????????????????
            with open('/home/wws/Downloads/referenceshinjuku.kml', 'w') as fp:
                fp.write(content)

        preTim = pullh2kml_eval_.GPS_Week_Second
