%% inputs
function Graphics (alphaArr)

global Ls Lss d
for i=1:length(alphaArr)
    alpha=alphaArr(i);
    Gs=[0,sin(alpha)*Ls];
    F=[Ls*cos(alpha),0];
    Gp=Gs+[0,d];
    A=Gs+Lss*[cos(alpha),-sin(alpha)];

%     Lp =sqrt(d^2+Lss^2+2*d*Lss*sin(alpha));
%     subplot(2,1,1)
    plot([Gs(1) F(1)],[Gs(2) F(2)])
    hold on
    plot([Gp(1) A(1)],[Gp(2) A(2)])
    axis equal
    hold off
    axis([0 10 0 10])
%     
%     subplot(2,1,2)
%     plot(i,Lp,'*')
%     hold on
    
    pause(0.1);
end
