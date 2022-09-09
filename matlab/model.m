%% model calculation

syms r l th phi

x = r*cos(th) + l*cos(th+phi);
y = r*sin(th) + l*sin(th+phi);
psi = th + phi;

syms x0 y0 p0
eqn = [x == x0, y == y0, psi == p0];
S = solve(eqn, [r, th, phi]);

S.r = simplify(S.r);
S.th = simplify(S.th);
S.phi = simplify(S.phi);

%% generate function

%{
mkdir("gen");
r_func = matlabFunction(S.r, 'File', 'gen/ik_r.m');
th_func = matlabFunction(S.th, 'File', 'gen/ik_theta.m');
phi_func = matlabFunction(S.phi, 'File', 'gen/ik_phi.m');
x_func = matlabFunction(x, 'File', 'gen/fk_x.m');
y_func = matlabFunction(y, 'File', 'gen/fk_y.m');
psi_func = matlabFunction(psi, 'File', 'gen/fk_psi.m');

cfg = coder.config('lib');
cfg.TargetLang = 'C++';
cfg.CppNamespace = 'model';
codegen -config cfg gen/ik_r.m -args {double(0), double(0), double(0), double(0)} ...
        gen/ik_theta.m -args {double(0), double(0), double(0), double(0)} ...
        gen/ik_phi.m -args {double(0), double(0), double(0), double(0)} ...
        gen/fk_x.m -args {double(0), double(0), double(0), double(0)} ...
        gen/fk_y.m -args {double(0), double(0), double(0), double(0)} ...
        gen/fk_psi.m -args {double(0), double(0)} ...
        -d ../src/kirin/gencode/ -std:c++11
%}

%% calc velocity model
% calc dx dy dpsi
syms t r_(t) th_(t) phi_(t)
v = [r, th, phi];
v_t = [r_(t), th_(t), phi_(t)];
x = subs(x, v, v_t);
y = subs(y, v, v_t);
psi = subs(psi, v, v_t);
dx = diff(x, t);
dy = diff(y, t);
dpsi = diff(psi, t);

% replace
syms dr dth dphi
d_old = [diff(r_(t), t), diff(th_(t), t), diff(phi_(t), t)];
d_new = [dr, dth, dphi];
dx = subs(dx, [d_old, v_t], [d_new, v]);
dy = subs(dy, [d_old, v_t], [d_new, v]);
dpsi = subs(dpsi, [d_old, v_t], [d_new, v]);

% calc ik
syms dx0 dy0 dp0
deqn = [dx == dx0, dy == dy0, dpsi == dp0];
dS = solve(deqn, [dr, dth, dphi]);
dS.dr = simplify(dS.dr);
dS.dth = simplify(dS.dth);
dS.dphi = simplify(dS.dphi);

%% generate function

dr_func = matlabFunction(dS.dr, 'File', 'gen/dr.m');
dth_func = matlabFunction(dS.dth, 'File', 'gen/dtheta.m');
dphi_func = matlabFunction(dS.dphi, 'File', 'gen/dphi.m');
% dx_func = matlabFunction(x, 'File', 'gen/fk_dx.m');
% dy_func = matlabFunction(y, 'File', 'gen/fk_dy.m');
% dpsi_func = matlabFunction(psi, 'File', 'gen/fk_dpsi.m');


cfg = coder.config('lib');
cfg.TargetLang = 'C++';
cfg.CppNamespace = 'model';
codegen -config cfg ...
        gen/dr.m -args {double(0), double(0), double(0), double(0), double(0), double(0)} ...
        gen/dtheta.m -args {double(0), double(0), double(0), double(0), double(0), double(0), double(0)} ...
        gen/dphi.m -args {double(0), double(0), double(0), double(0), double(0), double(0), double(0)} ...
        -d ../src/kirin/gencode/ -std:c++11


%% remove extra

for ext = ["mat", "o", "mk", "tmw", "dmw", "dmr", "a"]
    path = "../src/kirin/gencode/*."+ext;
    delete(path);
end
rmdir('../src/kirin/gencode/examples', 's');
rmdir('../src/kirin/gencode/interface', 's');



