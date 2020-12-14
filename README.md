# industrial-robots
Robots used in industry

## Dependencies

There is 1 dependency 'matrix-computations'.

```bash
https://github.com/PeterTadich/matrix-computations
```

## Installation

### Node.js

```bash
npm install https://github.com/PeterTadich/industrial-robots
```

### Google Chrome Web browser

No installation required for the Google Chrome Web browser.

## How to use

### Node.js

```js
import * as mcir from 'industrial-robots';
```

### Google Chrome Web browser

```js
import * as mcir from './mcir.mjs';
```
## Examples

### Node.js (server side)

Copy the following code to index.mjs

```js
//npm install https://github.com/PeterTadich/newton-euler https://github.com/PeterTadich/industrial-robots

import * as hlao from 'matrix-computations';
import * as mcnef from 'newton-euler';
import * as mcir from 'industrial-robots';

var mua = mcir.puma560(); //get the robot
var n = mua.DH.length; //number of links

//run Newton-Euler (recursive)
var kin; //kinematics
var trq; //torques

/*
//Newton-Euler (recursive) - base frame
//   - forward recursion
kin = mcnef.linkAccelerationsBF(mua);
//   - backward recursion
trq = mcnef.linkForcesBF(mua);
*/
//or
trq = mcnef.NewtonEulerRecursion(mua,"BF");

//print the torques
console.log("torques - base frame:");
for(var i=1;i<=n;i=i+1){
    console.log("torques: T" + i + " = " + trq[i].toFixed(4));
}

/*
//Newton-Euler (recursive) - current frame
//   - forward recursion
kin = mcnef.linkAccelerationsCF(mua);
//   - backward recursion
trq = mcnef.linkForcesCF(mua);
*/
//or
trq = mcnef.NewtonEulerRecursion(mua,"CF");

//print the torques
console.log("torques - current frame:");
for(var i=1;i<=n;i=i+1){
    console.log("torques: T" + i + " = " + trq[i].toFixed(4));
}
```

Then run:

```bash
npm init -y
npm install https://github.com/PeterTadich/newton-euler https://github.com/PeterTadich/industrial-robots
node index.mjs
```

If the above does not work modify the package.json file as follows:
Helpful ref: [https://stackoverflow.com/questions/45854169/how-can-i-use-an-es6-import-in-node-js](https://stackoverflow.com/questions/45854169/how-can-i-use-an-es6-import-in-node-js)

```js
"scripts": {
    "test": "echo \"Error: no test specified\" && exit 1",
    "start": "node --experimental-modules index.mjs"
  },
"type": "module",
```

```bash
npm start
```

Result:

```js
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
```