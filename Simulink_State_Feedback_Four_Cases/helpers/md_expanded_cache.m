function md_expanded_cache
% Short session-only build folders avoid Windows' 260-character path limit.
% Source models and report outputs remain in the project folder.
Simulink.fileGenControl('set', ...
    'CacheFolder',fullfile(tempdir,'md8sl_cache'), ...
    'CodeGenFolder',fullfile(tempdir,'md8sl_codegen'),'createDir',true);
end
