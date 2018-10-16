#ifndef MYZMQCLIENT_H
#define MYZMQCLIENT_H



#define ZMQ_NAME_CENTER "Rb_MsgCenter"
#define ZMQ_NAME_IMGRECO "Rb_ImageRecognition"
#define ZMQ_NAME_CAMSERV "Rb_CameraService"
#define ZMQ_NAME_AUDISERV "Rb_AudioService"
#define ZMQ_NAME_ROUTPLANMING "Rb_RoutePlanNavi"
#define ZMQ_NAME_PANTILT "Rb_PanTiltCtrl"
#define ZMQ_NAME_MANGSYS "Rb_ManagementSystem"
#define ZMQ_NAME_DRIVERCTRL "Rb_DriverCtrl"

#define ZMQ_NAME_ALLNODE "Rb_ALLNode"

#define ZMQ_STAND_TOPIC_DRIVERUPDATE "Rb_DriverUpdate"

#define MYGEARTRpcTopic "HeartBeat"

#define UPDATE_FUN_NAME "Dcupdataall"

#define DCOM1 "/dev/ttyS0" 
#define DCOM2 "/dev/ttyS1"
#define DCOM3 "/dev/ttyS2" 

#define CAN1 "can0"
#define CAN2 "can1"

// ssj 4-9
#include "../../robot2015_common/include/PackageMessage.pb.h"
#include "../../robot2015_common/include/ControllerMessage.pb.h"
#include "../../robot2015_common/include/ParamConfig.pb.h"
#include "../../robot2015_common/include/BMSMessage.pb.h"
#include "../../robot2015_common/include/AlarmMessage.pb.h"
#include "../../robot2015_common/include/ErrorMessage.pb.h"
#include "../../robot2015_common/include/BmsModeConfig.pb.h"

#endif // MYZMQCLIENT_H

