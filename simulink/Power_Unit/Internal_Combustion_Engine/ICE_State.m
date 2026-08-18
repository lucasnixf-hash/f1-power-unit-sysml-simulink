classdef ICE_State < Simulink.IntEnumType
    enumeration
        Off_Standby(0)
        Cranking(1)
        Idle(2)
        Running(3)
        Fuel_Cut(4)
        Limp_Home(5)
        Fail_Safe(6)
    end
    methods (Static)
        function retVal = getDefaultValue()
            retVal = ICE_State.Off_Standby;
        end
    end
end