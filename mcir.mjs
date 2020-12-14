import * as hlao from 'matrix-computations';

/*
%MATLAB
clear; close all; clc;
mdl_puma560
q = [0.6,0.19,0.75,0.91,0.11,0.71];
qd = [0.67,0.1,1.3,0.43,0.9,1.2];
qdd = [0.37,0.21,0.76,0.23,0.19,1.2];
% with friction
p560.rne(q,qd,qdd)
% no friction
rfn = p560.nofriction('all')
rfn.rne(q,qd,qdd)
*/

/*
//result:
torques - base frame:
torques: T1 = 29.8313
torques: T2 = 44.3561
torques: T3 = 6.0648
torques: T4 = 1.0745
torques: T5 = 1.0699
torques: T6 = 0.7957
torques - current frame:
torques: T1 = 29.8313
torques: T2 = 44.3561
torques: T3 = 6.0648
torques: T4 = 1.0745
torques: T5 = 1.0699
torques: T6 = 0.7957
*/

function puma560(){
    var qz = [0,0,0,0,0,0];
    var qr = [0,Math.PI/2.0,-Math.PI/2.0,0,0,0];
    var qs = [0,0,-Math.PI/2.0,0,0,0];
    var qn = [0,Math.PI/4.0,-Math.PI,0,Math.PI/4.0,0];
    
    //var v = [0.0,0.0,0.0,0.0,0.0,0.0];
    //var vd = [0.0,0.0,0.0,0.0,0.0,0.0];
    //var vdd = [0.0,0.0,0.0,0.0,0.0,0.0];
    var v = [0.6,0.19,0.75,0.91,0.11,0.71];
    var vd = [0.67,0.1,1.3,0.43,0.9,1.2];
    var vdd = [0.37,0.21,0.76,0.23,0.19,1.2];
    
    //inertia
    var I = [];
    I[0] = [[0.0,  0.0, 0.0],
            [0.0, 0.35, 0.0],
            [0.0,  0.0, 0.0]];
    I[1] = [[ 0.13,   0.0,   0.0],
            [  0.0, 0.524,   0.0],
            [  0.0,   0.0, 0.539]];
    I[2] = [[0.066,   0.0,    0.0],
            [  0.0, 0.086,    0.0],
            [  0.0,   0.0, 0.0125]];
    I[3] = [[ 0.0018,    0.0,    0.0],
            [    0.0, 0.0013,    0.0],
            [    0.0,    0.0, 0.0018]];
    I[4] = [[0.0003,    0.0,    0.0],
            [   0.0, 0.0004,    0.0],
            [   0.0,    0.0, 0.0003]];
    I[5] = [[0.00015,     0.0,   0.0],
            [    0.0, 0.00015,   0.0],
            [    0.0,     0.0, 4e-05]];
    
    var mua = {
        //Denavit-Hartenberg parameters
        //        a,            alpha,      d,               q
        DH: [
            [0.0000,      Math.PI/2.0, 0.0000, v[0]],
            [0.4318,              0.0, 0.0000, v[1]],
            [0.0203, -1.0*Math.PI/2.0, 0.1500, v[2]],
            [0.0000,      Math.PI/2.0, 0.4318, v[3]],
            [0.0000, -1.0*Math.PI/2.0, 0.0000, v[4]],
            [0.0000,              0.0, 0.0000, v[5]]
        ],
        //joint type
        jt:['R','R','R','R','R','R'],
        //joint kinematics
        v: v, //joint position (same as q, qd, qdd (generalized coordinates) for revolute joint)
        vd: vd, //joint velocity
        vdd: vdd, //joint acceleration
        //link kinematics
        w: [[[0.0],[0.0],[0.0]]], //link 0 (frame {0}) angular velocity (initial condition)
        wd: [[[0.0],[0.0],[0.0]]], //link 0 (frame {0}) angular acceleration (initial condition)
        wdr: [], //rotor
        pd: [], //link velocity
        pdd: [[[0.0],[0.0],[0.0]]], //link acceleration (initial condition)
        pcdd: [], //CoM acceleration
        R: [hlao.identity_matrix(3)],
        //14 dynamic parameters per joint (page 262, 263):
        m: [0.0,17.4,4.8,0.82,0.34,0.09], //(1) mass of the links
        mr: [0.0,0.0,0.0,0.0,0.0,0.0], //(1) mass of the rotor
        rc: [ //(3) components of the first moment of inertia (7.72) (does centre of mass change if 'd' changes?)
            [    [0.0],     [0.0],    [0.0]],
            [[-0.3638],   [0.006], [0.2275]],
            [[-0.0203], [-0.0141],   [0.07]],
            [    [0.0],   [0.019],    [0.0]],
            [    [0.0],     [0.0],    [0.0]],
            [    [0.0],     [0.0],  [0.032]]
            ],
        I: I, //(6) components of the inertia tensor in (7.73),
        Ir: [0.0002,0.0002,0.0002,0.000033,0.000033,0.000033], //(1) the moment of inertia of the rotor (IMPORTANT: need to check this)
        Fvi: [], //(1) viscous friction coefficient Fvi
        Fsi: [], //(1) Coulomb friction coefficient Fsi
        //link velocities derivation (page 108)
        r: [ //position of {i} w.r.t {i-1}
                [   [0.0],    [0.0], [0.0]], //config. @ qz
                [[0.4318],    [0.0], [0.0]],
                [[0.0203],  [-0.15], [0.0]],
                [   [0.0], [0.4318], [0.0]],
                [   [0.0],    [0.0], [0.0]],
                [   [0.0],    [0.0], [0.0]]
            ],
        //others
        //g: [[0.0],[-9.81],[0.0]], //   - define 'g' (gravity) direction.
        g: [[0.0],[0.0],[-9.81]], //   - define 'g' (gravity) direction.
        kr: [-62.61,107.8,-53.71,76.04,71.92,76.69], //   - gear reduction ratio. kr.
        Bm: [0.00148,0.000817,0.00138,0.0000712,0.0000826,0.0000367],
        Tc: [[0.395,-0.435],[0.126,-0.071],[0.132,-0.105],[0.0112,-0.0169],[0.00926,-0.0145],[0.00396,-0.0105]]
    };
    
    //setup r (link length)
    var Li = mua.DH;
    var n = mua.DH.length;
    var R = [];
    var T0 = []
    T0[0] = [
        [1.0,0.0,0.0,0.0],
        [0.0,1.0,0.0,0.0],
        [0.0,0.0,1.0,0.0],
        [0.0,0.0,0.0,1.0]
    ];
    var rc = [];
    var r = [];
    for(var i=1;i<=n;i=i+1){ //coordinate transforms between Frame {i} and Frame {i-1}
        //homogeneous transformation matrix:
        //note: Denavit-Hartenberg parameters for Link 1 are indexed at '0'
        var Aip = [ //Ai',i-1
            [Math.cos(Li[i-1][3]),-1.0*Math.sin(Li[i-1][3]), 0.0,         0.0],
            [Math.sin(Li[i-1][3]),     Math.cos(Li[i-1][3]), 0.0,         0.0],
            [                 0.0,                      0.0, 1.0,  Li[i-1][2]],
            [                 0.0,                      0.0, 0.0,         1.0]
        ];
        
        var Ai = [ //Ai,i'
            [      1.0,                  0.0,                      0.0,  Li[i-1][0]],
            [      0.0, Math.cos(Li[i-1][1]),-1.0*Math.sin(Li[i-1][1]),         0.0],
            [      0.0, Math.sin(Li[i-1][1]),     Math.cos(Li[i-1][1]),         0.0],
            [      0.0,                  0.0,                      0.0,         1.0]
        ];
        
        var Ai_ineg1 = hlao.matrix_multiplication(Aip,Ai);
        
        //rotation matrix
        R[i] = [
            [Ai_ineg1[0][0],Ai_ineg1[0][1],Ai_ineg1[0][2]],
            [Ai_ineg1[1][0],Ai_ineg1[1][1],Ai_ineg1[1][2]],
            [Ai_ineg1[2][0],Ai_ineg1[2][1],Ai_ineg1[2][2]]
        ];
        
        //transform
        T0[i] = hlao.matrix_multiplication(T0[i-1],Ai_ineg1);
        //console.log(T0[i]);
        
        //link length (defined in {0})
        r[i] = [
            [T0[i][0][3] - T0[i-1][0][3]],
            [T0[i][1][3] - T0[i-1][1][3]],
            [T0[i][2][3] - T0[i-1][2][3]]
        ];
        
        //link length ((defined in {i}))
        mua.r[i-1] = hlao.matrix_multiplication(
                    hlao.matrix_transpose(
                        [
                            //[T0[i-1][0][0],T0[i-1][0][1],T0[i-1][0][2]],
                            //[T0[i-1][1][0],T0[i-1][1][1],T0[i-1][1][2]],
                            //[T0[i-1][2][0],T0[i-1][2][1],T0[i-1][2][2]]
                            [T0[i][0][0],T0[i][0][1],T0[i][0][2]],
                            [T0[i][1][0],T0[i][1][1],T0[i][1][2]],
                            [T0[i][2][0],T0[i][2][1],T0[i][2][2]]
                        ]
                    ),
                    r[i]
                );
        //console.log(mua.r[i-1]);
    }
    
    return mua;
}

/*
%MATLAB
clear; close all; clc;
mdl_stanford
q = [0.6,0.19,0.75,0.91,0.11,0.71];
qd = [0.67,0.1,1.3,0.43,0.9,1.2];
qdd = [0.37,0.21,0.76,0.23,0.19,1.2];
% with friction
stanf.rne(q,qd,qdd)
% no friction
rfn = stanf.nofriction('all')
rfn.rne(q,qd,qdd)
*/

/*
//result:
torques - base frame:
torques: T1 = 7.6674
torques: T2 = 54.6007
torques: T3 = 67.4632
torques: T4 = 0.9348
torques: T5 = -2.3287
torques: T6 = 0.0244
torques - current frame:
torques: T1 = 7.6674
torques: T2 = 54.6007
torques: T3 = 67.4632
torques: T4 = 0.9348
torques: T5 = -2.3287
torques: T6 = 0.0244
*/

function stanford(){
    var qz = [0,0,0,0,0,0];
    
    //var v = [0.0,0.0,0.0,0.0,0.0,0.0];
    //var vd = [0.0,0.0,0.0,0.0,0.0,0.0];
    //var vdd = [0.0,0.0,0.0,0.0,0.0,0.0];
    var v = [0.6,0.19,0.75,0.91,0.11,0.71];
    var vd = [0.67,0.1,1.3,0.43,0.9,1.2];
    var vdd = [0.37,0.21,0.76,0.23,0.19,1.2];
    
    //inertia
    var I = [];
    I[0] = [[0.276,   0.0,   0.0],
            [  0.0, 0.255,   0.0],
            [  0.0,   0.0, 0.071]];
    I[1] = [[ 0.108,   0.0, 0.0],
            [   0.0, 0.018, 0.0],
            [   0.0,   0.0, 0.1]];
    I[2] = [[2.51,  0.0,   0.0],
            [ 0.0, 2.51,   0.0],
            [ 0.0,  0.0, 0.006]];
    I[3] = [[ 0.002,   0.0,   0.0],
            [   0.0, 0.001,   0.0],
            [   0.0,   0.0, 0.001]];
    I[4] = [[0.003,    0.0, 0.0],
            [  0.0, 0.0004, 0.0],
            [  0.0,    0.0, 0.0]];
    I[5] = [[0.013,   0.0,    0.0],
            [  0.0, 0.013,    0.0],
            [  0.0,   0.0, 0.0003]];
    
    var mua = {
        //Denavit-Hartenberg parameters
        //        a,            alpha,      d,               q
        DH: [
            [0.0000, -1.0*Math.PI/2.0, 0.4120,             v[0]],
            [0.0000,      Math.PI/2.0, 0.1540,             v[1]],
            [0.0000,              0.0,   v[2], -1.0*Math.PI/2.0],
            [0.0000, -1.0*Math.PI/2.0, 0.0000,             v[3]],
            [0.0000,      Math.PI/2.0, 0.0000,             v[4]],
            [0.0000,              0.0, 0.2630,             v[5]]
        ],
        //joint type
        jt:['R','R','P','R','R','R'],
        //joint kinematics
        v: v, //joint position (same as q, qd, qdd (generalized coordinates) for revolute joint)
        vd: vd, //joint velocity
        vdd: vdd, //joint acceleration
        //link kinematics
        w: [[[0.0],[0.0],[0.0]]], //link 0 (frame {0}) angular velocity (initial condition)
        wd: [[[0.0],[0.0],[0.0]]], //link 0 (frame {0}) angular acceleration (initial condition)
        wdr: [], //rotor
        pd: [], //link velocity
        pdd: [[[0.0],[0.0],[0.0]]], //link acceleration (initial condition)
        pcdd: [], //CoM acceleration
        R: [hlao.identity_matrix(3)],
        //14 dynamic parameters per joint (page 262, 263):
        m: [9.29,5.01,4.25,1.08,0.63,0.51], //(1) mass of the links
        mr: [0.0,0.0,0.0,0.0,0.0,0.0], //(1) mass of the rotor
        rc: [ //(3) components of the first moment of inertia (7.72) (does centre of mass change if 'd' changes?)
                [[0.0],[0.0175], [-0.1105]],
                [[0.0],[-1.054],     [0.0]],
                [[0.0],   [0.0],  [-6.447]],
                [[0.0], [0.092],  [-0.054]],
                [[0.0], [0.566],   [0.003]],
                [[0.0],   [0.0],   [1.554]]
            ],
        I: I, //(6) components of the inertia tensor in (7.73),
        Ir: [0.953,2.193,0.782,0.106,0.097,0.02], //(1) the moment of inertia of the rotor (IMPORTANT: need to check this)
        Fvi: [], //(1) viscous friction coefficient Fvi
        Fsi: [], //(1) Coulomb friction coefficient Fsi
        //link velocities derivation (page 108)
        r: [ //position of {i} w.r.t {i-1}
                [ [ 0 ], [ -0.412 ], [     0 ] ],
                [ [ 0 ], [  0.154 ], [     0 ] ],
                [ [ 0 ], [      0 ], [     0 ] ],
                [ [ 0 ], [      0 ], [     0 ] ],
                [ [ 0 ], [      0 ], [     0 ] ],
                [ [ 0 ], [      0 ], [ 0.263 ] ]
            ],
        //others
        //g: [[0.0],[-9.81],[0.0]], //   - define 'g' (gravity) direction.
        g: [[0.0],[0.0],[-9.81]], //   - define 'g' (gravity) direction.
        kr: [1.0,1.0,1.0,1.0,1.0,1.0], //   - gear reduction ratio. kr.
        Bm: [0.0,0.0,0.0,0.0,0.0,0.0],
        Tc: [[0.0,0.0],[0.0,0.0],[0.0,0.0],[0.0,0.0],[0.0,0.0],[0.0,0.0]]
    };
    
    //setup r (link length)
    var Li = mua.DH;
    var n = mua.DH.length;
    var R = [];
    var T0 = []
    T0[0] = [
        [1.0,0.0,0.0,0.0],
        [0.0,1.0,0.0,0.0],
        [0.0,0.0,1.0,0.0],
        [0.0,0.0,0.0,1.0]
    ];
    var rc = [];
    var r = [];
    for(var i=1;i<=n;i=i+1){ //coordinate transforms between Frame {i} and Frame {i-1}
        //homogeneous transformation matrix:
        //note: Denavit-Hartenberg parameters for Link 1 are indexed at '0'
        var Aip = [ //Ai',i-1
            [Math.cos(Li[i-1][3]),-1.0*Math.sin(Li[i-1][3]), 0.0,         0.0],
            [Math.sin(Li[i-1][3]),     Math.cos(Li[i-1][3]), 0.0,         0.0],
            [                 0.0,                      0.0, 1.0,  Li[i-1][2]],
            [                 0.0,                      0.0, 0.0,         1.0]
        ];
        
        var Ai = [ //Ai,i'
            [      1.0,                  0.0,                      0.0,  Li[i-1][0]],
            [      0.0, Math.cos(Li[i-1][1]),-1.0*Math.sin(Li[i-1][1]),         0.0],
            [      0.0, Math.sin(Li[i-1][1]),     Math.cos(Li[i-1][1]),         0.0],
            [      0.0,                  0.0,                      0.0,         1.0]
        ];
        
        var Ai_ineg1 = hlao.matrix_multiplication(Aip,Ai);
        
        //rotation matrix
        R[i] = [
            [Ai_ineg1[0][0],Ai_ineg1[0][1],Ai_ineg1[0][2]],
            [Ai_ineg1[1][0],Ai_ineg1[1][1],Ai_ineg1[1][2]],
            [Ai_ineg1[2][0],Ai_ineg1[2][1],Ai_ineg1[2][2]]
        ];
        
        //transform
        T0[i] = hlao.matrix_multiplication(T0[i-1],Ai_ineg1);
        //console.log(T0[i]);
        
        //link length (defined in {0})
        r[i] = [
            [T0[i][0][3] - T0[i-1][0][3]],
            [T0[i][1][3] - T0[i-1][1][3]],
            [T0[i][2][3] - T0[i-1][2][3]]
        ];
        
        //link length ((defined in {i}))
        mua.r[i-1] = hlao.matrix_multiplication(
                    hlao.matrix_transpose(
                        [
                            //[T0[i-1][0][0],T0[i-1][0][1],T0[i-1][0][2]],
                            //[T0[i-1][1][0],T0[i-1][1][1],T0[i-1][1][2]],
                            //[T0[i-1][2][0],T0[i-1][2][1],T0[i-1][2][2]]
                            [T0[i][0][0],T0[i][0][1],T0[i][0][2]],
                            [T0[i][1][0],T0[i][1][1],T0[i][1][2]],
                            [T0[i][2][0],T0[i][2][1],T0[i][2][2]]
                        ]
                    ),
                    r[i]
                );
        //console.log(mua.r[i-1]);
    }
    
    return mua;
}

export {
    puma560,
    stanford
};