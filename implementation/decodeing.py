
import struct, socket
from copy import copy 

class Parser(object):
 
    def __init__(self):
        self.version = (0, 0)   
        self._dataqueue = bytes() 
        self.host = "192.168.0.2" 
        secondary_port = 30001 #Secondary client interface on Universal Robots
        self._s_secondary = socket.create_connection((self.host, secondary_port), timeout=0.5)  # the timeout gives a chance to stop the thread
    
    def parse(self, data):
            """
            parse a packet from the UR socket and return a dictionary with the data
            """
            allData = {}
            
            while data:
                psize, ptype, pdata, data = self.analyze_header(data)  

                # print "We got packet with size %i and type %s" % (psize, ptype)
                if ptype == 16:
                    allData["SecondaryClientData"] = self._get_data(pdata, "!iB", ("size", "type"))
                    data = (pdata + data)[5:]  # This is the total size so we resend data to parser
                elif ptype == 0:
                    # this parses RobotModeData for versions >=3.0 (i.e. 3.0)
                    if psize == 38:
                        self.version = (3, 0)
                        allData['RobotModeData'] = self._get_data(pdata, "!IBQ???????BBdd", ("size", "type", "timestamp", "isRobotConnected", "isRealRobotEnabled", "isPowerOnRobot", "isEmergencyStopped", "isSecurityStopped", "isProgramRunning", "isProgramPaused", "robotMode", "controlMode", "speedFraction", "speedScaling"))
                    elif psize == 46:  # It's 46 bytes in 3.2
                        self.version = (3, 2)
                        allData['RobotModeData'] = self._get_data(pdata, "!IBQ???????BBdd", ("size", "type", "timestamp", "isRobotConnected", "isRealRobotEnabled", "isPowerOnRobot", "isEmergencyStopped", "isSecurityStopped", "isProgramRunning", "isProgramPaused", "robotMode", "controlMode", "speedFraction", "speedScaling", "speedFractionLimit"))
                    elif psize == 47:
                        self.version = (3, 5)
                        allData['RobotModeData'] = self._get_data(pdata, "!IBQ???????BBddc", ("size", "type", "timestamp", "isRobotConnected", "isRealRobotEnabled", "isPowerOnRobot", "isEmergencyStopped", "isSecurityStopped", "isProgramRunning", "isProgramPaused", "robotMode", "controlMode", "speedFraction", "speedScaling", "speedFractionLimit", "reservedByUR"))
                    else:
                        allData["RobotModeData"] = self._get_data(pdata, "!iBQ???????Bd", ("size", "type", "timestamp", "isRobotConnected", "isRealRobotEnabled", "isPowerOnRobot", "isEmergencyStopped", "isSecurityStopped", "isProgramRunning", "isProgramPaused", "robotMode", "speedFraction"))
                elif ptype == 1:
                    tmpstr = ["size", "type"]
                    for i in range(0, 6):
                        tmpstr += ["q_actual%s" % i, "q_target%s" % i, "qd_actual%s" % i, "I_actual%s" % i, "V_actual%s" % i, "T_motor%s" % i, "T_micro%s" % i, "jointMode%s" % i]

                    allData["JointData"] = self._get_data(pdata, "!iB dddffffB dddffffB dddffffB dddffffB dddffffB dddffffB", tmpstr)

                elif ptype == 4:
                    if self.version < (3, 2):
                        allData["CartesianInfo"] = self._get_data(pdata, "iBdddddd", ("size", "type", "X", "Y", "Z", "Rx", "Ry", "Rz"))
                    else:
                        allData["CartesianInfo"] = self._get_data(pdata, "iBdddddddddddd", ("size", "type", "X", "Y", "Z", "Rx", "Ry", "Rz", "tcpOffsetX", "tcpOffsetY", "tcpOffsetZ", "tcpOffsetRx", "tcpOffsetRy", "tcpOffsetRz"))
                elif ptype == 5:
                    allData["LaserPointer(OBSOLETE)"] = self._get_data(pdata, "iBddd", ("size", "type"))
                elif ptype == 3:

                    if self.version >= (3, 0):
                        fmt = "iBiibbddbbddffffBBb"     # firmware >= 3.0
                    else:
                        fmt = "iBhhbbddbbddffffBBb"     # firmware < 3.0

                    allData["MasterBoardData"] = self._get_data(pdata, fmt, ("size", "type", "digitalInputBits", "digitalOutputBits", "analogInputRange0", "analogInputRange1", "analogInput0", "analogInput1", "analogInputDomain0", "analogInputDomain1", "analogOutput0", "analogOutput1", "masterBoardTemperature", "robotVoltage48V", "robotCurrent", "masterIOCurrent"))  # , "masterSafetyState" ,"masterOnOffState", "euromap67InterfaceInstalled"   ))
                elif ptype == 2:
                    allData["ToolData"] = self._get_data(pdata, "iBbbddfBffB", ("size", "type", "analoginputRange2", "analoginputRange3", "analogInput2", "analogInput3", "toolVoltage48V", "toolOutputVoltage", "toolCurrent", "toolTemperature", "toolMode"))
                elif ptype == 9:
                    continue  # This package has a length of 53 bytes. It is used internally by Universal Robots software only and should be skipped.
                elif ptype == 8 and self.version >= (3, 2):
                    allData["AdditionalInfo"] = self._get_data(pdata, "iB??", ("size", "type", "teachButtonPressed", "teachButtonEnabled"))
                elif ptype == 7 and self.version >= (3, 2):
                    allData["ForceModeData"] = self._get_data(pdata, "iBddddddd", ("size", "type", "x", "y", "z", "rx", "ry", "rz", "robotDexterity"))
                # elif ptype == 8:
                #     allData["varMessage"] = self._get_data(pdata, "!iBQbb iiBAcAc", ("size", "type", "timestamp", "source", "robotMessageType", "code", "argument", "titleSize", "messageTitle", "messageText"))
                # elif ptype == 7:
                #     allData["keyMessage"] = self._get_data(pdata, "!iBQbb iiBAcAc", ("size", "type", "timestamp", "source", "robotMessageType", "code", "argument", "titleSize", "messageTitle", "messageText"))

                elif ptype == 20:
                    tmp = self._get_data(pdata, "!iB Qbb", ("size", "type", "timestamp", "source", "robotMessageType"))
                    if tmp["robotMessageType"] == 3:
                        allData["VersionMessage"] = self._get_data(pdata, "!iBQbb bAbBBiAb", ("size", "type", "timestamp", "source", "robotMessageType", "projectNameSize", "projectName", "majorVersion", "minorVersion", "svnRevision", "buildDate"))
                    elif tmp["robotMessageType"] == 6:
                        allData["robotCommMessage"] = self._get_data(pdata, "!iBQbb iiAc", ("size", "type", "timestamp", "source", "robotMessageType", "code", "argument", "messageText"))
                    elif tmp["robotMessageType"] == 1:
                        allData["labelMessage"] = self._get_data(pdata, "!iBQbb iAc", ("size", "type", "timestamp", "source", "robotMessageType", "id", "messageText"))
                    elif tmp["robotMessageType"] == 2:
                        allData["popupMessage"] = self._get_data(pdata, "!iBQbb ??BAcAc", ("size", "type", "timestamp", "source", "robotMessageType", "warning", "error", "titleSize", "messageTitle", "messageText"))
                    elif tmp["robotMessageType"] == 0:
                        allData["messageText"] = self._get_data(pdata, "!iBQbb Ac", ("size", "type", "timestamp", "source", "robotMessageType", "messageText"))
                    elif tmp["robotMessageType"] == 8:
                        allData["varMessage"] = self._get_data(pdata, "!iBQbb iiBAcAc", ("size", "type", "timestamp", "source", "robotMessageType", "code", "argument", "titleSize", "messageTitle", "messageText"))
                    elif tmp["robotMessageType"] == 7:
                        allData["keyMessage"] = self._get_data(pdata, "!iBQbb iiBAcAc", ("size", "type", "timestamp", "source", "robotMessageType", "code", "argument", "titleSize", "messageTitle", "messageText"))
                    elif tmp["robotMessageType"] == 5:
                        allData["keyMessage"] = self._get_data(pdata, "!iBQbb iiAc", ("size", "type", "timestamp", "source", "robotMessageType", "code", "argument", "messageText"))
                    """ else:
                
                     print("Message type parser not implemented %s" % (tmp)) 
                    """ 
                """  else:
                    print("Unknown packet type %s with size %s" % (ptype, psize))
                """
            return allData



    def _get_data(self, data, fmt, names):
        """
        fill data into a dictionary
            data is data from robot packet
            fmt is struct format, but with added A for arrays and no support for numerical in fmt
            names args are strings used to store values
        """ 
        #tmpdata = copy(data) 
        
        # Copy data has been removed, if enabled it would create a copy of the data, which is not needed.
        tmpdata = data
        fmt = fmt.strip()  # space may confuse us
        d = dict()
        i = 0
        j = 0
        while j < len(fmt) and i < len(names):
            f = fmt[j]
            if f in (" ", "!", ">", "<"):
                j += 1
            elif f == "A":  # we got an array
                # first we need to find its size
                if j == len(fmt) - 2:  # we are last element, size is the rest of data in packet
                    arraysize = len(tmpdata)
                else:  # size should be given in last element
                    asn = names[i - 1]
                    if not asn.endswith("Size"): 
                        None
                        #print("Error, array without size ! %s %s" % (asn, i))
                    else:
                        arraysize = d[asn]
                d[names[i]] = tmpdata[0:arraysize]
                # print "Array is ", names[i], d[names[i]]
                tmpdata = tmpdata[arraysize:]
                j += 2
                i += 1
            else:
                fmtsize = struct.calcsize(fmt[j])
                # print "reading ", f , i, j,  fmtsize, len(tmpdata)
                #if len(tmpdata) < fmtsize:  # seems to happen on windows
                   # print("Error, length of data smaller than advertized: ", len(tmpdata), fmtsize, "for names ", names, f, i, j)
                d[names[i]] = struct.unpack("!" + f, tmpdata[0:fmtsize])[0]
                # print names[i], d[names[i]]
                tmpdata = tmpdata[fmtsize:]
                j += 1
                i += 1
        return d  
    
    def get_header(self, data):
        return struct.unpack("!iB", data[0:5])

    def analyze_header(self, data):
            """
            Read first 5 bytes and return complete packet
            """
            if len(data) < 5: 
                None
                #print("Packet size %s smaller than header size (5 bytes)" % len(data))
            else:
                psize, ptype = self.get_header(data)
                #if psize < 5:
                   # print("Error, declared length of data smaller than its own header(5): ", psize)
                    #elif psize > len(data):
                    #print("Error, length of data smaller (%s) than declared (%s)" % (len(data), psize))
            return psize, ptype, data[:psize], data[psize:]

    def find_first_packet(self, data):
        """
        Find the first complete packet in a string
        returns None if none found
        """
        counter = 0
        limit = 10
        while True:
            if len(data) >= 5:
                psize, ptype = self.get_header(data)
                if psize < 5 or psize > 2000 or ptype != 16:
                    data = data[1:]
                    counter += 1
                    if counter > limit:
                       # print("tried %s times to find a packet in data, advertised packet size: %s, type: %s" % (counter, psize, ptype))
                        #print("Data length: %s" % (len(data)))
                        limit = limit * 10
                elif len(data) >= psize:
                    #print("Got packet with size %s and type %s" % (psize, ptype))
                    #if counter:
                       # print("Remove %s bytes of garbage at begining of packet" % (counter))
                    # ok we we have somehting which looks like a packet"
                    return (data[:psize], data[psize:])
                else:
                    # packet is not complete
                    # print("Packet is not complete, advertised size is %s, received size is %s, type is %s"% ( psize, len(data), ptype))
                    return None
            else:
                # self.logger.debug("data smaller than 5 bytes")
                return None  
            
    def _get_packge(self):
        """
        returns something that looks like a packet, nothing is guaranted
        """
        while True:
            # self.logger.debug("data queue size is: {}".format(len(self._dataqueue)))
            ans = self.find_first_packet(self._dataqueue[:])
            if ans:
                self._dataqueue = ans[1]
                # self.logger.debug("found packet of size {}".format(len(ans[0])))
                return ans[0]
            else:
                # self.logger.debug("Could not find packet in received data")
                tmp = self._s_secondary.recv(1024)
                self._dataqueue += tmp 

program=Parser()    
all_data=program.parse(program._get_packge())   

    
""" 
if __name__ == '__main__' :  
    while  True: 
        all_data=program.parse(program._get_packge())   
        #print(all_data)
        if True:  
            joints = all_data['JointData'] 
            robot_pose= all_data['CartesianInfo']  
            robot_coordinates =robot_pose["X"], robot_pose["Y"], robot_pose["Z"], robot_pose["Rx"], robot_pose["Ry"], robot_pose["Rz"]  
            robot_offset=robot_pose["tcpOffsetX"], robot_pose["tcpOffsetY"],robot_pose["tcpOffsetZ"] , robot_pose["tcpOffsetRx"], robot_pose["tcpOffsetRy"], robot_pose["tcpOffsetRz"] 
            robot_joints=all_data["JointData"]['q_actual0'],all_data["JointData"]['q_actual1'],all_data["JointData"]['q_actual2'],all_data["JointData"]['q_actual3'],all_data["JointData"]['q_actual4'],all_data["JointData"]['q_actual5']
                        
           
            print(robot_joints) 
            #print(robot_coordinates)
            #print("Angles =  %s %s %s %s %s %s " % (math.degrees(joints['q_actual0']),math.degrees(joints['q_actual1']),math.degrees(joints['q_actual2']),math.degrees(joints['q_actual3']),math.degrees(joints['q_actual4']),math.degrees(joints['q_actual5']))) 

"""
