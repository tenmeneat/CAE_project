function [seq, sig] = superpose_stress(F_loc, C)
%SUPERPOSE_STRESS 단위하중 응력계수로 텐서 중첩 → 부호 있는 von Mises (블록 5).
%   [seq, sig] = SUPERPOSE_STRESS(F_loc, C)
%   F_loc - N×3 LBJ 하중 (로어암 로컬, 블록 3 F_BJ(:,:,i)) [N]
%   C     - 응력계수 표 (stress_coeffs_beam 또는 read_stress_coeffs_csv), C.coef H×6×3 [Pa/N]
%   seq   - N×H 부호 있는 von Mises [MPa], 부호 = 정수압 성분(트레이스) 부호 (0이면 +)
%   sig   - N×6×H 응력텐서 [MPa] (두 번째 출력을 요청할 때만 만듦)
%   텐서 성분을 먼저 더한 뒤 등가응력을 낸다 — von Mises끼리 더하지 않는다.

H = size(C.coef, 1);
N = size(F_loc, 1);
seq = zeros(N, H);
if nargout > 1, sig = zeros(N, 6, H); end
for h = 1:H
    s = F_loc * squeeze(C.coef(h, :, :)).' / 1e6;    % N×6 [MPa]
    vm = sqrt(0.5*((s(:,1)-s(:,2)).^2 + (s(:,2)-s(:,3)).^2 + (s(:,3)-s(:,1)).^2) + 3*sum(s(:,4:6).^2, 2));
    sg = sign(sum(s(:, 1:3), 2));
    sg(sg == 0) = 1;
    seq(:, h) = sg .* vm;
    if nargout > 1, sig(:, :, h) = s; end
end
end
