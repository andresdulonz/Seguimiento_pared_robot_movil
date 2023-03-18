clear all
close all
clc

bot = Bot_dr12();

S = 20;

v = 0.2;
w = 0;

tic;
while toc<=S
    bot.Set_Joint_Velocity(v,w);
    
    d = bot.Read_Sensors();
    
    disp('Sensor readings:')
    disp(d)
    
    if bot.Is_Done()
        disp('Line detected!')
        
        v = -0.2;
    end
end
