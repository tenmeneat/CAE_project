function trk = load_centerline(csv, ds)
%LOAD_CENTERLINE 시나리오 B용 중심선 로더 (w_r, w_l 포함). load_raceline과 같은 형식.
if nargin < 2, ds = 1; end
trk = load_raceline(csv, ds);
end
