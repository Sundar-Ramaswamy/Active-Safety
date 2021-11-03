%close all force
%clear all
clc

%% INIT
% Import libraries
currentFolder = strsplit(pwd, '\');
parentFolder = fullfile(currentFolder{1:end-1});
addpath(genpath(parentFolder));

% Data folders
SOURCE.experimentDataFolder = fullfile(parentFolder, 'data');
SOURCE.dataParticipantsFolder = fullfile(SOURCE.experimentDataFolder, ...
    'data_participants');

% Coding schema
SOURCE.CodingSchemaFile = fullfile(SOURCE.experimentDataFolder,...
    'codingSchema.json');
SOURCE.EventType = {'A'};

VC = VideoController.runInstance(SOURCE);