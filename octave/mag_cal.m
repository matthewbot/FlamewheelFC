clear all;
close all;
clc;

format long;

data = csvread('data.csv');
data = data(:, 7:9);
data = data(1:50:length(data),:);
data(:, 1) *= -1;
data(:, 3) *= -1;

figure;
scatter3(data(:,1),data(:,2),data(:,3),'r');

A = zeros(size(data,1), 9);
A(:,1) = data(:,1).^2;
A(:,2) = data(:,2).^2;
A(:,3) = data(:,3).^2;
A(:,4) = 2*data(:,1).*data(:,2);
A(:,5) = 2*data(:,1).*data(:,3);
A(:,6) = 2*data(:,2).*data(:,3);
A(:,7) = data(:,1);
A(:,8) = data(:,2);
A(:,9) = data(:,3);

B = ones(size(data,1), 1);

X = (A'*A)\(A'*B);
Xorig = X;


error = A(:,1).*X(1) + A(:,2).*X(2) + A(:,3).*X(3) + ...
    A(:,4).*X(4) + A(:,5).*X(5) + A(:,6).*X(6) + ...
    A(:,7).*X(7) +A(:,8).*X(8) + A(:,9).*X(9);

error = error - 1;
mean(abs(error));
max(abs(error));
mse = mean(error.^2)


%Find the center of ellipsoid
a = [X(1) X(4) X(5); X(4) X(2) X(6); X(5) X(6) X(3)];
b = [X(7); X(8); X(9)];

[P1,a2,P2] = svd(a);

b2 = (b'*P1)';

x0 = (b2 ./ diag(a2))*.5;

shift = P1*x0;
 rot = P1;

c = 1 + (x0(1))^2*a2(1,1) + (x0(2))^2*a2(2,2) + (x0(3))^2*a2(3,3);

%Find the 1/2 lenght of all "major" axis of the ellipsoid
Major1 = sqrt(c/a2(1,1));
Major2 = sqrt(c/a2(2,2));
Major3 = sqrt(c/a2(3,3));

correction = rot'*diag(1./[Major1 Major2 Major3])*rot
shift

gooddata = zeros(size(data));
for i = 1:rows(data)
  gooddata(i, :) = (correction * (data(i, :)' + shift))';
endfor

figure;
scatter3(gooddata(:,1),gooddata(:,2),gooddata(:,3),'r');


