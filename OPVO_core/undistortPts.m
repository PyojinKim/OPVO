function [uv_u] = undistortPts(uv_d, cam)

% normalize uv_distorted
p_n_d = inv(cam.K) * [uv_d;1];
p_n_d = p_n_d / p_n_d(3);
p_n_d = p_n_d(1:2);
p_n_u = p_n_d;


while (1)
    
    error = distort_normal(p_n_u, cam) - p_n_d;
    p_n_u = p_n_u - error;
    
    if ( sum(abs(error)) < 1e-9 )
        break;
    end
end


% denormalize uv_undistorted
uv_u = cam.K * [p_n_u;1];
uv_u = uv_u / uv_u(3);
uv_u = uv_u(1:2);


end


% function normalize(x, y)
% y_n = (y-cy)/fy;
% x_n = (x-cx)/fx ? skew_c*y_n;
% return (x_n, y_n);
% end
%
%
% function denormalize(x, y)
% x_p = fx*(x + skew_c*y) + cx;
% y_p = fy*y + cy;
% return (x_p, y_p);
% end


function [p_n_d] = distort_normal(p_n_u, cam)

x = p_n_u(1);
y = p_n_u(2);
k1 = cam.k1;
k2 = cam.k2;
k3 = 0;
p1 = cam.p1;
p2 = cam.p2;

r2 = x*x + y*y;
radial_d = 1 + k1*r2 + k2*r2*r2 + k3*r2*r2*r2;
x_d = radial_d * x + 2*p1*x*y + p2*(r2 + 2*x*x);
y_d = radial_d * y + p1*(r2 + 2*y*y) + 2*p2*x*y;

p_n_d = [x_d; y_d];

end