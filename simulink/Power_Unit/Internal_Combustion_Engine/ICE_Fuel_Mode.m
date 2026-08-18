% ice_enum_defs.m —— 在 ICE_sim.slx 的模型 InitFcn 里调用，供
% ICE_STM / ICE_Strategy_Mapper / Fuel_System 共同引用，避免各自重复定义。

classdef ICE_Fuel_Mode < Simulink.IntEnumType
    enumeration
        Normal(0)
        Lift_and_Coast(1)
        Fuel_Saving(2)
    end
    methods (Static)
        function retVal = getDefaultValue()
            retVal = ICE_Fuel_Mode.Normal;
        end
    end
end