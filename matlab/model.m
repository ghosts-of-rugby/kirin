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
for ext = ["mat", "o", "mk", "tmw", "dmw", "dmr", "a"]
    path = "../src/kirin/gencode/*."+ext;
    delete(path);
end
rmdir('../src/kirin/gencode/examples', 's');
rmdir('../src/kirin/gencode/interface', 's');
