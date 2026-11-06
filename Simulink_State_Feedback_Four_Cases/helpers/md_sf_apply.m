function r=md_sf_apply(model,cfg)
% Refresh this model's derived data and every bound display alias from one file.
% Called on load and before Run; the models no longer own independent settings.
if nargin<2, cfg=sf_parameters; end
r=md_sf_prepare(cfg); name=get_param(model,'Name');
if contains(name,'_linear_'), r.mode='linear'; else, r.mode='nonlinear'; end
workspace=get_param(model,'ModelWorkspace'); wasDirty=get_param(model,'Dirty');
assignin(workspace,'r',r);
blocks=find_system(model,'LookUnderMasks','all','Type','Block'); count=0;
for k=1:numel(blocks)
    b=blocks{k}; data=get_param(b,'UserData');
    if ~isstruct(data) || ~isfield(data,'mdSharedBinding'), continue; end
    binding=data.mdSharedBinding;
    value=eval(binding.expression); % expressions installed in this owned model
    assignin(workspace,binding.alias,value);
    if ~strcmp(get_param(b,binding.parameter),binding.alias)
        set_param(b,binding.parameter,binding.alias);
    end
    count=count+1;
end
assert(count>0,'Shared bindings are missing: use the updated teaching models.');
% Saved configuration already refers to r.test/r.p.cfg.sim. Do not reset it
% here: a deliberate sim(...,'StopTime',...) override must remain effective.
set_param(model,'Dirty',wasDirty);
end

