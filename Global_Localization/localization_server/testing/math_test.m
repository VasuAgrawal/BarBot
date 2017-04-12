syms x11 x12 x21 x22 % X values for 2 items
assume(x11, 'real');
assume(x12, 'real');
assume(x21, 'real');
assume(x22, 'real');

X = [x11 x12 x21 x22];

syms r11 r12 r21 r22 % Reference distances
assume(r11 >= 0);
assume(r12 >= 0);
assume(r21 >= 0);
assume(r22 >= 0);
R = [r11 r12 r21 r22];

x1 = [x11 x12];
x2 = [x21 x22];
assume(x1, 'real');
assume(x2, 'real');

guess = [norm(x1 - x1), norm(x2 - x1), norm(x1 - x2), norm(x2 - x2)];
loss = norm(R - guess);
diff(loss, x11);