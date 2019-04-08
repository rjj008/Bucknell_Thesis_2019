clc;clear all;close all;

%% init variables
WantedFreqOfData=2500;
TimeTakeDataForce = 20;
MotorSpinUpSpeed = 180;
MotorMinSpeed = 140;MotorMaxSpeed = 200; %motor min and max speed by PWM
ArduinoSpeeds = [MotorMinSpeed:20:MotorMaxSpeed];%speeds between 0 and 255 for input into PWN in arduino
p=1;
AllForceData=[];
WantedFreqOfDataPres=10;%freq of wind tunnel presure and temp to take
TimeTakeDataPres=5;%time to take pressure and temp data
StillTakeData=true();
windTunnelSpeedHzStart=0;%start speed of wind tunnel
gg=1;
nn=1;
TrialinitData=[];
takeDataBool=true();

%% set up analog channel in daq for force data
DAQObjForce = daq.createSession('ni');
[ForceChanX,ForceindexX]=addAnalogInputChannel(DAQObjForce,'cDAQ2Mod1', 0, 'Voltage');%pressure tranducer voltage channel
[ForceChanY,ForceindexY]=addAnalogInputChannel(DAQObjForce,'cDAQ2Mod1', 1, 'Voltage');%pressure tranducer voltage channe
[ForceChanZ,ForceindexZ]=addAnalogInputChannel(DAQObjForce,'cDAQ2Mod1', 2, 'Voltage');%pressure tranducer voltage channe
DAQObjForce.Rate = WantedFreqOfData;
DAQObjForce.DurationInSeconds = TimeTakeDataForce;


%% set up pressure, temp NI daq session
DAQObjPress = daq.createSession('ni');
[Presschan,PresIndex]=addAnalogInputChannel(DAQObjPress,'cDAQ1Mod2', 0, 'Voltage');%pressure tranducer voltage channel
[Tempchan,TempIndex]=addAnalogInputChannel(DAQObjPress,'cDAQ1Mod1', 0, 'Thermocouple');%pressure tranducer voltage channel
TC = DAQObjPress.Channels(TempIndex);
TC.ThermocoupleType = 'K';
TC.Units = 'Celsius';
DAQObjPress.Rate = WantedFreqOfDataPres;
DAQObjPress.DurationInSeconds = TimeTakeDataPres;


%% daq objct to control wind tunnel speed
DAQObjWindTun =  daq.createSession('ni');
addAnalogOutputChannel(DAQObjWindTun,'cDAQ1Mod3',0,'Voltage');%wind tunnel output voltage

outputSingleScan(DAQObjWindTun,windTunnelSpeedHzStart);%set wind tunnel speed

%
%% Open Arduino Motor Port
MotorComPortNum=4;
MotorCurrentCom=strcat('COM',string(MotorComPortNum));
MotorarduinoCom = serial(MotorCurrentCom,'BaudRate',9600);
fopen(MotorarduinoCom);
OpenArduinoSpinMotor2(0,MotorarduinoCom);%makes motor speed zero to start off with
% %pause(1);

%OpenArduinoSpinMotor2(MotorSpinUpSpeed,MotorarduinoCom);%makes motor speed zero to start off with

%% encoder port vars and open
ArduinoEncoderCOMPort = 3;
KeepGoing = true();
ArduinoCurrentComEnc=strcat('COM',string(ArduinoEncoderCOMPort));
ArduinoEncoderSerial = serial(ArduinoCurrentComEnc,'BaudRate',9600 );
ArduinoEncoderSerial.Terminator = 'CR';
fopen(ArduinoEncoderSerial);

%% Start to take data
while (StillTakeData)
    TrialinitData=[];
    windTunnelSpeedHz = input('What is the speed of the wind tunnel in Hz[10-60 HZ]?:');
    BallRotationAngle = input('What is the ball rotation angle (deg)?');
    BallSpinCase = input('Is this a YES ball spin(0) or NO ball spin(1)?:');
    if windTunnelSpeedHz =='end'
        break;
    end
    if BallSpinCase =='end'
        break;
    end
    
    
    outputSingleScan(DAQObjWindTun,interp1([10,60],[0,10],windTunnelSpeedHz));%set wind tunnel speed
    input('wind tunnel being spun up... should I continue to case initiilization zero spin?');
    
    disp('taking pressure and temp data');
    PressData1 = DAQObjPress.startForeground;%[pressure(volts,temp(C)]
    
    disp('taking encoder speed');
    fprintf(ArduinoEncoderSerial,'7;');%gets arduino encoder speed
    ArduinoSpeed=fscanf(ArduinoEncoderSerial);
    SpeedMotor1(1,1) = str2double(ArduinoSpeed);%speed of motor in RPM
    
    disp('taking force data');
    [ForceDataVolt1,Forcetime1] = DAQObjForce.startForeground;%[Fx,Fy,Fx] this is is in volts
    
    disp('taking encoder speed data again');
    fprintf(ArduinoEncoderSerial,'7;');%gets arduino encoder speed
    ArduinoSpeed=fscanf(ArduinoEncoderSerial);
    SpeedMotor1(2,1) = str2double(ArduinoSpeed);%speed of motor in RPM
    
    ForceDataLb1 = interp1([-5,0,5],[-7.5,0,7.5],ForceDataVolt1);%gets force data to force data in lb
    
    ZeroDataTogeth(1:length(ForceDataLb1),1:3)=ForceDataLb1;%put force data in first 3 colms
    ZeroDataTogeth(1:length(Forcetime1),4)=Forcetime1;%put force time data in 4
    ZeroDataTogeth(1:length(PressData1),5:6)=PressData1;%put press, temp data in colsms 5 and 6
    ZeroDataTogeth(1:length(SpeedMotor1),7)=SpeedMotor1;%put pmotor sped data 7 colm
    ZeroDataTogeth(1:length([1]),8)=0;%put motor input PWM sped data 8 colm
    ZeroDataTogeth(1:length(ForceDataLb1),9)=BallRotationAngle*ones(length(ForceDataLb1),1);%put ball rotation angle in colm 9
    
     input('zero spin data fininshed. should I continue to spin case initiilization?');

    
    
    
    if BallSpinCase == 1 %NO BALL SPIN
        [PressData,PresTime]=DAQObjPress.startForeground();
        [ForceDataVolt,Forcetime] = DAQObjForce.startForeground;
        [RowForceDataVolt,ColmForceDataVolt]=size(ForceDataVolt);
       
       ForceDataLb = interp1([-5,0,5],[-7.5,0,7.5],ForceDataVolt);%gets force data to force data in lb       
    end
    
    if BallSpinCase == 0 % YES ball spin
        OpenArduinoSpinMotor2(MotorSpinUpSpeed,MotorarduinoCom);%spin up baseball
        disp('Spinning up baseball');
        xxx=input('should i go to trial adruino speeds?');
        
        for i=1:length(ArduinoSpeeds)
            disp(ArduinoSpeeds(i));
            OpenArduinoSpinMotor2(ArduinoSpeeds(i),MotorarduinoCom);%sets motor speed
            pause(22) %allows motor to spin up to speed
            %xxx=input('should i take data now');
            disp('taking pressure and temp data');
            PressData = DAQObjPress.startForeground;%[pressure(volts,temp(C)]
            
            disp('taking encoder speed');
            fprintf(ArduinoEncoderSerial,'7;');%gets arduino encoder speed
            ArduinoSpeed=fscanf(ArduinoEncoderSerial);
            SpeedMotor(1,1) = str2double(ArduinoSpeed);%speed of motor in RPM
            
            disp('taking force data');
            [ForceDataVolt,Forcetime] = DAQObjForce.startForeground;%[Fx,Fy,Fx] this is is in volts
            
            disp('taking encoder speed data again');
            fprintf(ArduinoEncoderSerial,'7;');%gets arduino encoder speed
            ArduinoSpeed=fscanf(ArduinoEncoderSerial);
            SpeedMotor(2,1) = str2double(ArduinoSpeed);%speed of motor in RPM
            
            ForceDataLb = interp1([-5,0,5],[-7.5,0,7.5],ForceDataVolt);%gets force data to force data in lb
            
            DataTogeth(1:length(ForceDataLb),1:3)=ForceDataLb;%put force data in first 3 colms
            DataTogeth(1:length(Forcetime),4)=Forcetime;%put force time data in 4
            DataTogeth(1:length(PressData),5:6)=PressData;%put press, temp data in colsms 5 and 6
            DataTogeth(1:length(SpeedMotor),7)=SpeedMotor;%put pmotor sped data 7 colm
            DataTogeth(1:length(ArduinoSpeeds(i)),8)=ArduinoSpeeds(i);%put motor input PWM sped data 8 colm
            DataTogeth(1:length(ForceDataLb),9)=BallRotationAngle*ones(length(ForceDataLb),1);%put ball rotation angle in colm 9

            
            TrialinitData =[TrialinitData;DataTogeth];%[Fx(lb),Fy{lb),Fz(lb),force time(s),Press(volts),Temp(C),MotorSprrd(RPM),Input PWM motor (0-255), anle baseball rotation(deg)]
            
        end
        
        TrialinitData =[ZeroDataTogeth;TrialinitData];
    end
     
    
    % set motor spped to whatever it should be in between trials
    % OpenArduinoSpinMotor2(0,MotorarduinoCom);%sets motor speed
    
    %[Arduino PWM,WindTunnelHz,Fx(lb),Fy(lb),Fz(lb),Tx(lb*in),Ty(lb*in),...
    % Tz(lb*in),Speedmotor(RPM),Pressure(raw voltage),Temp(C)]
    %AllData =[TrialinitData];
    
    
    % export data to csv file
    if takeDataBool
        FileNameData = strcat(date,"_WindTunnelSpeed_",num2str(windTunnelSpeedHz)...
            ,"_BallSpin_",num2str(MotorMinSpeed),"-",num2str(MotorMaxSpeed),".csv");
        
        dlmwrite(FileNameData,TrialinitData,'-append');
    end
    
    
    ballEndSpeed = input('should I stop ball (1) or run it at the low speed(0) for next trial?');
    
        if ballEndSpeed==1
             OpenArduinoSpinMotor2(0,MotorarduinoCom)
            
        else OpenArduinoSpinMotor2(ArduinoSpeeds(1),MotorarduinoCom)
        end

        
end

%close relavent ports
fclose(MotorarduinoCom);
fclose(ArduinoEncoderSerial);
release(DAQObjPress);
release(DAQObjWindTun);
release(DAQObjForce);













