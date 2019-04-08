clc;clear all;close all;

%% init variables
WantedFreqOfData=5;
TimeTakeDataForce = 5;
AllForceData=[];
StillTakeData=true();
TrialinitData=[];
takeDataBool=true();
StillTakeData=true();

%% set up analog channel in daq for force data
DAQObjForce = daq.createSession('ni');
[ForceChanX,ForceindexX]=addAnalogInputChannel(DAQObjForce,'cDAQ2Mod1', 0, 'Voltage');%pressure tranducer voltage channel
[ForceChanY,ForceindexY]=addAnalogInputChannel(DAQObjForce,'cDAQ2Mod1', 1, 'Voltage');%pressure tranducer voltage channe
[ForceChanZ,ForceindexZ]=addAnalogInputChannel(DAQObjForce,'cDAQ2Mod1', 2, 'Voltage');%pressure tranducer voltage channe
DAQObjForce.Rate = WantedFreqOfData;
DAQObjForce.DurationInSeconds = TimeTakeDataForce;



%%take calibrationData
while (StillTakeData)
    In1TakeData = input('should I still take data?(1=yes and 0=no)');
    
    if In1TakeData == 0 %breaks while loop if no more data to be taken
        break;
    end

    BallAngle =input('what is the ball angle in degrees?');

    [ForceDataVolt,Forcetime] = DAQObjForce.startForeground;
    
    ForceDataLb = interp1([-5,0,5],[-7.5,0,7.5],ForceDataVolt);%gets force data to force data in lb
    
    AveragedForces=mean(ForceDataLb);
    
    DataTogeth = [AveragedForces,BallAngle];
    %DataTogeth = [Fx,Fy,Fz,Angle]
    
    FileNameData = strcat(date,"_Calibration_BallRotationAngle.csv");
    dlmwrite(FileNameData,DataTogeth,'-append');
end

release(DAQObjForce);