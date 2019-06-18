function g = leastSQUARE(y, u, m1, m2)

n = length(y);
U1 = zeros(n, -m1);
U2 = zeros(n, m2);

for ii = 0:-m1 - 1
  a = u(-m1+1-ii:end);
  U1(1:length(a), ii+1) = a;
end

for ii = 0:m2
  U2(1+ii:end, ii+1) = u(1:end-ii);
end

U = [U1, U2];

%g = (U' * U) \ U' * y;
g=inv(U' * U)*U' * y;