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

r_func = matlabFunction(S.r, 'File', 'gen/r.m');
th_func = matlabFunction(S.th, 'File', 'gen/theta.m');
phi_func = matlabFunction(S.phi, 'File', 'gen/phi.m');

cfg = coder.config('lib');
cfg.TargetLang = 'C++';
cfg.CppNamespace = 'model';
codegen -config cfg gen/r.m -args {double(0), double(0), double(0), double(0)} ...
        gen/theta.m -args {double(0), double(0), double(0), double(0)} ...
        gen/phi.m -args {double(0), double(0), double(0), double(0)} ...
        -d ../src/kirin/gencode/ -std:c++11
for ext = ["mat", "o", "mk", "tmw", "dmw", "dmr", "a"]
    path = "../src/kirin/gencode/*."+ext;
    delete(path);
end
rmdir('../src/kirin/gencode/examples', 's');
rmdir('../src/kirin/gencode/interface', 's');
