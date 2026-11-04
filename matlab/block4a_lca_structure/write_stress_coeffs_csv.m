function write_stress_coeffs_csv(file, Cs)
%WRITE_STRESS_COEFFS_CSV 응력계수 표 CSV 쓰기 (블록 4a → 5). Cs: C 구조체 배열 (p별).
%   형식 (한 행 = p × 케이스 × 핫스팟, 로어암 로컬 좌표, [Pa/N]): p,load_case,hotspot,s11,s22,s33,s12,s23,s13
%   COMSOL 결과도 이 형식으로 내보내면 read_stress_coeffs_csv로 바로 읽는다.

rows = {};
for C = Cs(:)'
    for k = 1:numel(C.cases)
        for h = 1:height(C.hot)
            rows(end+1, :) = [{C.p, C.cases{k}, C.hot.name{h}}, num2cell(C.coef(h, :, k))]; %#ok<AGROW>
        end
    end
end
T = cell2table(rows, 'VariableNames', {'p', 'load_case', 'hotspot', 's11', 's22', 's33', 's12', 's23', 's13'});
writetable(T, file);
end
