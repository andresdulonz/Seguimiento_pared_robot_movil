classdef Bot_dr12 < handle
    properties (Access = private)
        CoppeliaSim = [];
        ClientID = [];
        
        dr12 = [];
        motorL = [];
        motorR = [];

        Si = [];
        LeftSensor = [];
        RightSensor = [];
        
        vl = -0.7; vu = 0.7;
        wl = -2.5; wu = 2.5;
        
        R = 0.086;
        L = 0.1635/2;
    end
    
    methods
        function obj = Bot_dr12()
            obj.CoppeliaSim = remApi('remoteApi');
            obj.CoppeliaSim.simxFinish(-1);
            disp('Connecting to robot....')
            obj.ClientID = obj.CoppeliaSim.simxStart('127.0.0.1',19999,true,true,5000,5); 
            
            if obj.ClientID==0
                [~,obj.motorL] = obj.CoppeliaSim.simxGetObjectHandle(obj.ClientID,'/dr12/leftJoint_',obj.CoppeliaSim.simx_opmode_oneshot_wait);
                [~,obj.motorR] = obj.CoppeliaSim.simxGetObjectHandle(obj.ClientID,'/dr12/rightJoint_',obj.CoppeliaSim.simx_opmode_oneshot_wait);

                [~,obj.Si(1)] = obj.CoppeliaSim.simxGetObjectHandle(obj.ClientID,'/dr12/SB_L',obj.CoppeliaSim.simx_opmode_oneshot_wait);
                [~,obj.Si(2)] = obj.CoppeliaSim.simxGetObjectHandle(obj.ClientID,'/dr12/SF_L',obj.CoppeliaSim.simx_opmode_oneshot_wait);
                [~,obj.Si(3)] = obj.CoppeliaSim.simxGetObjectHandle(obj.ClientID,'/dr12/SM',obj.CoppeliaSim.simx_opmode_oneshot_wait);
                [~,obj.Si(4)] = obj.CoppeliaSim.simxGetObjectHandle(obj.ClientID,'/dr12/SF_R',obj.CoppeliaSim.simx_opmode_oneshot_wait);
                [~,obj.Si(5)] = obj.CoppeliaSim.simxGetObjectHandle(obj.ClientID,'/dr12/SB_R',obj.CoppeliaSim.simx_opmode_oneshot_wait);
                
                [~,obj.LeftSensor] = obj.CoppeliaSim.simxGetObjectHandle(obj.ClientID,'/dr12/LeftSensor',obj.CoppeliaSim.simx_opmode_oneshot_wait);
                [~,obj.RightSensor] = obj.CoppeliaSim.simxGetObjectHandle(obj.ClientID,'/dr12/RightSensor',obj.CoppeliaSim.simx_opmode_oneshot_wait);
                
                pause(1)

                disp(' done.')
            else
                error(' robot not connected...')
            end
        end

        function obj = Set_Joint_Velocity (obj,v,w)
            if obj.CoppeliaSim.simxGetConnectionId(obj.ClientID)==1
                v = max(v,obj.vl); v = min(v,obj.vu); 
                w = max(w,obj.wl); w = min(w,obj.wu);
                
                Wr = (2*v+w*obj.L)/(2*obj.R);
                Wl = (2*v-w*obj.L)/(2*obj.R);
                
                obj.CoppeliaSim.simxSetJointTargetVelocity(obj.ClientID,obj.motorL,Wl,obj.CoppeliaSim.simx_opmode_streaming);
                obj.CoppeliaSim.simxSetJointTargetVelocity(obj.ClientID,obj.motorR,Wr,obj.CoppeliaSim.simx_opmode_streaming);
            else
                error(' connection lost...')
            end
        end

        function d = Read_Sensors(obj)
            d = -1*ones(1,5);
            
            for i=1:5
                aux_1 = 1;
                
                while aux_1==1
                    if obj.CoppeliaSim.simxGetConnectionId(obj.ClientID)==1
                        [aux_1,aux_2,D] = obj.CoppeliaSim.simxReadProximitySensor(obj.ClientID,obj.Si(i),obj.CoppeliaSim.simx_opmode_streaming);
                        
                        if aux_2==1
                            d(i) = D(3);
                        end
                    else
                        error(' connection lost...')
                    end
                end
            end
        end
        
        function s = Is_Done(obj)
            aux = ones(2,1);
            
            while sum(aux)~=0
                if obj.CoppeliaSim.simxGetConnectionId(obj.ClientID)==1
                    [aux(1),dl] = obj.CoppeliaSim.simxReadVisionSensor(obj.ClientID,obj.LeftSensor,obj.CoppeliaSim.simx_opmode_streaming);
                    [aux(2),dr] = obj.CoppeliaSim.simxReadVisionSensor(obj.ClientID,obj.RightSensor,obj.CoppeliaSim.simx_opmode_streaming);
                    
                    sl = (dl==0);
                    sr = (dr==0);
                    
                    if sl==1 && sr==1
                        s = 1;
                    else
                        s = 0;
                    end   
               else
                    error(' connection lost...')
                end
            end
        end
        
        function s = Connection(obj)
            s = obj.CoppeliaSim.simxGetConnectionId(obj.ClientID);
        end

        function Stop_Simulation (obj)
            obj.CoppeliaSim.simxStopSimulation(obj.ClientID,obj.CoppeliaSim.simx_opmode_oneshot_wait);
        end
    end
end