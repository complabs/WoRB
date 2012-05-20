%% make - Recompiles WoRB mexFunction
%
%  Filename: make.m
%  Revision: 0.5
%  Date:     2012-05-11
%  Author:   Mikica B Kocic

function make( varargin )

    % How to configure?
    % Setup GLUT_INC, GLUT_LIB, GLUT_BIN
    % Setup DEBUG flag
    
    DEBUG = false;   %#ok<*UNRCH>
    
    % ------------------------------------------------------------------------------------
    %% GLUT switches configuration (architecture dependent)
    
    arch = computer('arch');
    
    if isequal( arch, 'win32' )
        GLUT_INC = '-Ifreeglut/include';
        GLUT_LIB = './freeglut/lib/freeglut.lib';
        GLUT_BIN = './freeglut/bin/freeglut.dll';
    elseif isequal( arch, 'win64' )
        GLUT_INC = '-Ifreeglut/include';
        GLUT_LIB = './freeglut/lib/x64/freeglut.lib';
        GLUT_BIN = './freeglut/bin/x64/freeglut.dll';
    else
        % If you have installed freeglut-devel package, then:
        %   1) GLUT_INC and GLUT_BIN should be empty 
        %   2) GLUT_LIB should be '-lglut -lGLU -lGL'
        GLUT_INC = '';
        GLUT_LIB = '-lglut -lGLU -lGL';
        GLUT_BIN = '';
    end

    % ------------------------------------------------------------------------------------
    %% Mex compiler & linker flags
    
    if DEBUG
        params.cflags = '-g -DDEBUG'; 
        params.lflags = '-g';
    else
        params.cflags = '-O';
        params.lflags = '';
    end

    % Folders with source, object and binary files
    params.src   = 'src';
    params.obj   = 'obj';
    params.bin   = 'bin';
    params.libs  = GLUT_LIB;    % Libraries
    
    % File extensions
    params.mexext  =  [ '.', mexext ];
    if isunix
        params.objext = '.o';
    else
        params.objext = '.obj';
    end
   
    % Rebuild all
    params.rebuild = nargin >= 1 && isequal( varargin{1}, 'rebuild' );

    % Clean object directory
    params.clean = nargin >= 1 && isequal( varargin{1}, 'clean' );

    % Specifics for GLUT dependable files
    params2 = params;
    if ~isempty( GLUT_INC )
        params2.cflags = [ params.cflags, ' ', GLUT_INC ];
    end

    if params.clean
        files = [ params.obj, filesep, '*', params.objext ];
        disp([ 'delete ', files ]);
        delete( files );
        return
    end
    
    % ------------------------------------------------------------------------------------
    %% Dependencies: WoRB library
    
    recompile( params, 'Constants.cpp', ...
        'Constants.h', 'Quaternion.h' ...
        );
    recompile( params, 'CollisionDetection.cpp', ...
        'WoRB.h', 'Constants.h', 'Quaternion.h', 'QTensor.h', 'Geometry.h', ...
        'RigidBody.h', 'Collision.h', 'CollisionResolver.h'...
        );
    recompile( params, 'ImpulseMethod.cpp', ...
        'WoRB.h', 'Constants.h', 'Quaternion.h', 'QTensor.h', 'Geometry.h', ...
        'RigidBody.h', 'Collision.h', 'CollisionResolver.h'...
        );
    recompile( params, 'PositionProjections.cpp', ...
        'WoRB.h', 'Constants.h', 'Quaternion.h', 'QTensor.h', 'Geometry.h', ...
        'RigidBody.h', 'Collision.h', 'CollisionResolver.h'...
        );
    recompile( params, 'WoRB.cpp', ...
        'WoRB.h', 'Constants.h', 'Quaternion.h', 'QTensor.h', 'Geometry.h', ...
        'RigidBody.h', 'Collision.h', 'CollisionResolver.h'...
        );
    recompile( params, 'Platform.cpp', ...
        'Utilities.h' ...
        );

    % ------------------------------------------------------------------------------------
    %% Dependencies: WoRB test-bed (mexFunction)

    recompile( params2, 'Utilities.cpp', ...
        'WoRB.h', 'Constants.h', 'Quaternion.h', 'QTensor.h', 'Geometry.h', ...
        'RigidBody.h', 'Collision.h', 'CollisionResolver.h', ...
        'Utilities.h', 'WoRB_TestBed.h' ...
        );
    recompile( params2, 'WoRB_TestBed.cpp', ...
        'WoRB.h', 'Constants.h', 'Quaternion.h', 'QTensor.h', 'Geometry.h', ...
        'RigidBody.h', 'Collision.h', 'CollisionResolver.h', ...
        'Utilities.h', 'WoRB_TestBed.h' ...
        );
    recompile( params2, 'mexFunction.cpp', ...
        'WoRB.h', 'Constants.h', 'Quaternion.h', 'QTensor.h', 'Geometry.h', ...
        'RigidBody.h', 'Collision.h', 'CollisionResolver.h', ...
        'Utilities.h', 'WoRB_TestBed.h', 'mexWoRB.h' ...
        );

    % ------------------------------------------------------------------------------------
    %% Dependencies: linker

    paramsLink = params;
    if isequal( arch, 'win64' )
        paramsLink.bin = [ params.bin, filesep, 'win64' ];
    end
    
    relinked = relink( paramsLink, 'WoRB', ...
        'mexFunction', ...
        'Constants', ...
        'CollisionDetection', ...
        'ImpulseMethod', ...
        'PositionProjections', ...
        'WoRB', ...
        'Platform', ...
        'Utilities', ...
        'WoRB_TestBed' ...
        );

    if relinked && ~isempty( GLUT_BIN )
        % copyfile( GLUT_BIN, params.bin )
    end

    addpath( genpath( params.bin ) );

    % ------------------------------------------------------------------------------------
end

%% Recompiles object file that is dependend on varargin{} source files.
% The first argument in varagin should contain name of the main source file. 

function varargout = recompile( params, varargin )

    if nargin < 2
        error( 'At least one source file should be specified.' );
    end

    import java.io.*

    rebuild = false;

    [ ~, fname, ~ ] = fileparts( varargin{1} );
    
    obj = File([ params.obj, filesep, fname, params.objext ]);

    if ~obj.exists () || params.rebuild
        % Turn on recompile flag
        rebuild = true;
    else
        % Check input file by file if it is more recent than the output
        for i = 1:length(varargin)
            src = File([ params.src, filesep, varargin{i} ]);
            if ~src.exists ()
                % Report missing source files
                error([ 'Source file ', varargin{i}, ' does not exist.' ]);
            elseif src.lastModified() > obj.lastModified ()
                % Source file is more recent, turn on recompile flag
                rebuild = true;
            end
        end
    end

    if rebuild
        cmd = [ 'mex ', params.cflags,  ' -outdir ', params.obj, ...
                ' -c ', params.src, filesep, varargin{1} ];
        disp( cmd )
        eval( cmd )
    end
    
    if nargout >= 1 
        varargout{1} = rebuild;
    end
end

%% Relinks mexfile that is dependend on varargin{} object files.

function varargout = relink( params, mexfile, varargin )

    if nargin < 3
        error( 'At least one object file should be specified.' );
    end

    import java.io.*

    rebuild = false;

    binName = [ params.bin, filesep, mexfile, params.mexext ];
    bin = File( binName );

    objList = '';
    if ~bin.exists () || params.rebuild
        % Turn on recompile flag
        rebuild = true;
        for i = 1:length(varargin)
            objName = [ params.obj, filesep, varargin{i}, params.objext ];
            objList = [ objList,  ' ...', char(10), ' ', objName, ]; %#ok<AGROW>
        end
    else
        % Check input file by file if it is more recent than the output
        for i = 1:length(varargin)
            objName = [ params.obj, filesep, varargin{i}, params.objext ];
            objList = [ objList,  ' ...', char(10), ' ', objName, ]; %#ok<AGROW>
            obj = File( objName );
            if ~obj.exists ()
                % Report missing source files
                error([ 'Object file ', varargin{i}, ' does not exist.' ]);
            elseif obj.lastModified() > bin.lastModified ()
                % Source file is more recent, turn on recompile flag
                rebuild = true;
            end
        end
    end

    if rebuild
        cmd = [ 'mex ', params.lflags, ' -output ', binName, objList, ...
                ' ...', char(10), ' ', params.libs ];
        disp( cmd )
        eval( cmd )
    end
    
    if nargout >= 1 
        varargout{1} = rebuild;
    end
end
