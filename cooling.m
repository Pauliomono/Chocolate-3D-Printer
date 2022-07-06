clear
clc
close all

t = linspace(0,120,1000);

c_p = 1007*1000;
rho = 1.225;
V = .002*.02*.02;
A = .02*.02 + 4*.002*.02;
h = 60;

tau = c_p*rho*V/A/h;

T_o = 26;
T_i = 40;

T = T_o + (T_i - T_o)*exp(-t/tau);

plot(t,T)
hold on

h = 10;

tau = c_p*rho*V/A/h;

T_o = 26;
T_i = 40;

T = T_o + (T_i - T_o)*exp(-t/tau);

plot(t,T,'--')

xlabel('Time (s)')
ylabel('T ($\circ$C)','interpreter','latex')
set(gca,'fontname','Times New Roman','fontsize',14)
legend('Forced Convection','Natural Convection','location','northeast','interpreter','latex')

