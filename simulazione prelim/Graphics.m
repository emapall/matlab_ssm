%% inputs
function Graphics (alphaArr, Fp, Fsx, Fsy)

global Ls Lss d rocketMass;
for i=1:length(alphaArr)
    alpha=alphaArr(i);
    Gs=[0,sin(alpha)*Ls];
    F=[Ls*cos(alpha),0];
    Gp=Gs+[0,d];
    A=Gs+Lss*[cos(alpha),-sin(alpha)];

%     Lp =sqrt(d^2+Lss^2+2*d*Lss*sin(alpha));
%     subplot(2,1,1)
%     plot([Gs(1) F(1)],[Gs(2) F(2)])
%     hold on
%     plot([Gp(1) A(1)],[Gp(2) A(2)])
%     axis equal
%     hold off
%     axis([0 10 0 10])
%     
%% plotting forces
    Lp =sqrt(d^2+Lss^2+2*d*Lss*sin(alpha));
    phi = acos((d^2+Lp^2-Lss^2)/Lp/d/2); % cosine theorem: Lss^2 = d^2+Lp^2-2*Lp*d*cos(phi)
%     subplot(2,1,2)
    plot(nan, nan); hold on;
    quiver(0,0,Fp*sin(phi),-Fp*cos(phi));
    quiver(0,0,Fsx,Fsy);
    quiver(0,0,0,-10*rocketMass);
    plot(0,0,"kx");
%     disp(rad2deg(phi));
    axis equal;
%     axis([-2e5 2e5 -2e5 2e5]);
    hold off;
    
    if(sign(Fp)<0)
        disp("COMPRESSIONE");
    else
        disp("TRAZIONE");
    end
    pause(0.1);
end
