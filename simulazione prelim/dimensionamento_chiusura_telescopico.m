x_min = min(x);
p_sta_max = pb(x==x_min); % = pa in the same moment

L_min = Lp(x==x_min);

disp("Lunghezza dello scorrimento interno=Lungh camera olio:");
disp(Lp_extended-L_min);

disp("Scorrimento pistone flottante nella camera piccola");
disp(xIn-x_min);

disp("Pressione statica di chiusra, ATM");
disp(p_sta_max/1e5);

disp("Raggi a e b,cm");
disp(sqrt([Aa, Ab]/pi)*100);