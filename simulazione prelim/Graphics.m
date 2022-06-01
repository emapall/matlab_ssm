%% inputs
function Graphics (alphaArr, Fp, Fsx, Fsy)

global Ls Lss d rocketMass;
for i=1:length(alphaArr)
    alpha=alphaArr(i);
    Gs=[0,sin(alpha)*Ls]; % secondary strut joint
    F=[Ls*cos(alpha),0]; % footpad
    Gp=Gs+[0,d]; % primary strut joint
    A=Gs+Lss*[cos(alpha),-sin(alpha)]; % primary-secondary intersection

%     Lp =sqrt(d^2+Lss^2+2*d*Lss*sin(alpha));
%     subplot(2,1,1)
    plot([Gs(1) F(1)],[Gs(2) F(2)])
    hold on
    plot([Gp(1) A(1)],[Gp(2) A(2)])
    axis equal
%     hold off
    axis([-2 10 -2 10])
%     
%% plotting forces
    Lp = sqrt(d^2+Lss^2+2*d*Lss*sin(alpha));
    phi = acos((d^2+Lp^2-Lss^2)/Lp/d/2); % cosine theorem: Lss^2 = d^2+Lp^2-2*Lp*d*cos(phi)
%     subplot(2,1,2)
%     plot(nan, nan); hold on;
    forcesDownscale=1e-5;
    quiver(Gp(1),Gp(2),Fp(i)*sin(phi),-Fp(i)*cos(phi),forcesDownscale);
    quiver(Gs(1),Gs(2),Fsx(i),Fsy(i),forcesDownscale);
    quiver(Gp(1)-1.8,Gp(2)+2,0,-10*rocketMass,forcesDownscale);

%     plot(0,0,"kx");
%     disp(rad2deg(phi));
%     axis equal;
%     axis([-2e5 2e5 -2e5 2e5]);
    hold off;
    
    if(sign(Fp(i))<0)
        disp("COMPRESSIONE");
    else
        disp("TRAZIONE");
    end
    pause(0.1);
end
