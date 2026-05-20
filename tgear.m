%=====================================================
%               tgear.m                      
%                                            
%  Author : Ying Huo                         
%                                            
% power command vs. thtl. relationship used
% in F-16 model
% 功率指令与油门杆位置之间的关系
% 油门杆在 77% 时功率为 64.94×0.77≈50%
%=====================================================

function tgear_value  = tgear ( thtl )

if ( thtl <= 0.77 )
    tgear_value = 64.94 * thtl;
else
    tgear_value = 217.38 * thtl - 117.38;
end

    