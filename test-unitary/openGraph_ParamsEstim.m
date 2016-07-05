% Read graphs of parameter estimation erros
close all;
clear all;
clc;

filenameformat = '/Figures/160630_fcost_analysis/eparam_gna_%d.fig'; % file name formate to same graphs of parameters estimation erros using the cost function fcost1
% filenameformat = '/Figures/160509_fcost_analysis/GNA_v2/eparam_fmincon_%d.fig'; % file name formate to same graphs of parameters estimation erros using the cost function fcost1


for k=1:20
    fname = sprintf(filenameformat,k);
    openfig([pwd fname],'new','visible')
end
