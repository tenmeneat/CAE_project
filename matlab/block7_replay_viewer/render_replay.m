function render_replay(i3_mat_path, out_mp4_path, opts)
%RENDER_REPLAY 리플레이 영상 렌더링 (블록 7, 발표용). 스켈레톤 — 미구현.
%   RENDER_REPLAY(i3_mat_path, out_mp4_path, opts)
%   입력: 인터페이스 I3 (replay_<track>_<ctrl>.mat) — t,s,x,y,psi,v,ax,ay,F_BJ,sigma_hot,D_cum
%   출력: VideoWriter로 1080p 30fps MP4.
%
%   W1부터 더미 I3 데이터로 트랙맵/차량 마커/동기 그래프 뼈대를 먼저 만들 것 (분석 완료를 기다리지 않음).
%   구현 뼈대: patch(EdgeColor='interp') 색칠 트랙, hgtransform 차량, xline 커서, drawnow->getframe->writeVideo.
%   주의: 그래프는 원본 해상도로 미리 그리고 커서만 이동 (다운샘플링하면 피크 소실).

error('render_replay:NotImplemented', 'TODO: 블록 7 구현 — 더미 I3 데이터로 뼈대부터 시작 가능');
end
