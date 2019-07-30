(function(){
	var getCubicBezierTimingFunction;
	
	(function(){
		/**
		 * https://github.com/gre/bezier-easing
		 * BezierEasing - use bezier curve for transition easing function
		 * by Gaëtan Renaudeau 2014 - 2015 – MIT License
		 */

		// These values are established by empiricism with tests (tradeoff: performance VS precision)
		var NEWTON_ITERATIONS = 4;
		var NEWTON_MIN_SLOPE = 0.001;
		var SUBDIVISION_PRECISION = 0.0000001;
		var SUBDIVISION_MAX_ITERATIONS = 10;

		var kSplineTableSize = 11;
		var kSampleStepSize = 1.0 / (kSplineTableSize - 1.0);

		var float32ArraySupported = typeof Float32Array === 'function';

		function A (aA1, aA2) { return 1.0 - 3.0 * aA2 + 3.0 * aA1; }
		function B (aA1, aA2) { return 3.0 * aA2 - 6.0 * aA1; }
		function C (aA1)      { return 3.0 * aA1; }

		// Returns x(t) given t, x1, and x2, or y(t) given t, y1, and y2.
		function calcBezier (aT, aA1, aA2) { return ((A(aA1, aA2) * aT + B(aA1, aA2)) * aT + C(aA1)) * aT; }

		// Returns dx/dt given t, x1, and x2, or dy/dt given t, y1, and y2.
		function getSlope (aT, aA1, aA2) { return 3.0 * A(aA1, aA2) * aT * aT + 2.0 * B(aA1, aA2) * aT + C(aA1); }

		function binarySubdivide (aX, aA, aB, mX1, mX2) {
		  var currentX, currentT, i = 0;
		  do {
		    currentT = aA + (aB - aA) / 2.0;
		    currentX = calcBezier(currentT, mX1, mX2) - aX;
		    if (currentX > 0.0) {
		      aB = currentT;
		    } else {
		      aA = currentT;
		    }
		  } while (Math.abs(currentX) > SUBDIVISION_PRECISION && ++i < SUBDIVISION_MAX_ITERATIONS);
		  return currentT;
		}

		function newtonRaphsonIterate (aX, aGuessT, mX1, mX2) {
		 for (var i = 0; i < NEWTON_ITERATIONS; ++i) {
		   var currentSlope = getSlope(aGuessT, mX1, mX2);
		   if (currentSlope === 0.0) {
		     return aGuessT;
		   }
		   var currentX = calcBezier(aGuessT, mX1, mX2) - aX;
		   aGuessT -= currentX / currentSlope;
		 }
		 return aGuessT;
		}

		getCubicBezierTimingFunction = function bezier (mX1, mY1, mX2, mY2) {
		  if (!(0 <= mX1 && mX1 <= 1 && 0 <= mX2 && mX2 <= 1)) {
		    throw new Error('bezier x values must be in [0, 1] range');
		  }

		  // Precompute samples table
		  var sampleValues = float32ArraySupported ? new Float32Array(kSplineTableSize) : new Array(kSplineTableSize);
		  if (mX1 !== mY1 || mX2 !== mY2) {
		    for (var i = 0; i < kSplineTableSize; ++i) {
		      sampleValues[i] = calcBezier(i * kSampleStepSize, mX1, mX2);
		    }
		  }

		  function getTForX (aX) {
		    var intervalStart = 0.0;
		    var currentSample = 1;
		    var lastSample = kSplineTableSize - 1;

		    for (; currentSample !== lastSample && sampleValues[currentSample] <= aX; ++currentSample) {
		      intervalStart += kSampleStepSize;
		    }
		    --currentSample;

		    // Interpolate to provide an initial guess for t
		    var dist = (aX - sampleValues[currentSample]) / (sampleValues[currentSample + 1] - sampleValues[currentSample]);
		    var guessForT = intervalStart + dist * kSampleStepSize;

		    var initialSlope = getSlope(guessForT, mX1, mX2);
		    if (initialSlope >= NEWTON_MIN_SLOPE) {
		      return newtonRaphsonIterate(aX, guessForT, mX1, mX2);
		    } else if (initialSlope === 0.0) {
		      return guessForT;
		    } else {
		      return binarySubdivide(aX, intervalStart, intervalStart + kSampleStepSize, mX1, mX2);
		    }
		  }

		  return function BezierEasing (x) {
		    if (mX1 === mY1 && mX2 === mY2) {
		      return x; // linear
		    }
		    // Because JavaScript number are imprecise, we should guarantee the extremes are right.
		    if (x === 0) {
		      return 0;
		    }
		    if (x === 1) {
		      return 1;
		    }
		    return calcBezier(getTForX(x), mY1, mY2);
		  };
		};
	})()

	/**
	# Function: bind

	Creates a new function that calls the passed function with the `this` 
	keyword set to the passed value.

	Arguments to the created function are passed to the passed function.

	## Parameters:

	*	fn - (Function) - The function to be called.
	*	thisArg - (mixed) - The value to be passed as the this parameter to the 
		target function when the bound function is called.

	## Returns:

	A copy of the given function with the specified this value.
	*/
	function bind(fn, thisArg){
		return function(){
			fn.apply(thisArg, [].slice.call(arguments, 0));
		};
	}

	/**
	# Function: addEvent

	Attaches an event listener to a DOM element.

	## Parameters:

	*	element - (DOMElement) - The element to attach the event to.
	*	event - (String) - The name of the event, without the "on" prefix.
	*	handler - (Function) - The function to attach as an event listener.
	*/
	var addEvent = function(element, event, handler){
		if (element.addEventListener){
			element.addEventListener(event, handler);
		}
		else if (element.attachEvent){
			element.attachEvent('on' + event, handler);
		}
	};

	/**
	# Function: forEach

	Executes a provided function once for each array element.

	## Parameters:

	*	ary - (Array) - The array to loop over.
	*	callback - (function) - the function to call once for each element in 
		the array.
	*	thisArg - (mixed) - Optional. A value to use as `this` when executing 
		the callback.

	## Callback

	The callback is called with the arguments:

	*	value - (mixed) - An item in the array.
	*	index - (number) - The index of the item in the array.
	*	ary - (Array) - The array.
	*/
	function forEach(ary, callback, thisArg){
		thisArg = thisArg || null;

		for (var i=0, len=ary.length; i<len; i++){
			callback.call(thisArg, ary[i], i, ary);
		}
	}

	/**
	# Function: loadImg

	load the image with the passed url.

	## Parameters:

	*	url - (String) - The url to the image.
	*	callback - (Function) - The function to call when the image has loaded.

	## Callback

	The callback is called with any error as the first argument. If the image 
	was loaded successfully the first argument is `null` and the img element is 
	the second.
	*/
	function loadImg(url, callback){
		var img = document.createElement('img');

		img.onload = function(){
			img.onload = null;
			img.onerror = null;
			callback(null, img);
		};

		img.error = function(err){
			img.onload = null;
			img.onerror = null;
			callback(err);
		}

		img.src = url;
	}

	/**
	# Function: awaitAll

	Calls a function with a callback parameter mutiple times, calling the passed 
	callback once all callbacks has resolved.

	## Parameters:

	*	fn - (Function) - A function that accepts a callback as the last argument.
	*	allArgs - (Array<Array>) - An array containing an array of arguments for 
		each call to be made to the fn function.
	*	callback - (Function) - The function to call once all calls to `fn` 
		resolve or there is an error.
	*	thisArg - (mixed) - Optional. A value to use as `this` in the callback.

	## Callback

	The callback is called with any error as the first argument. If the all the 
	calls to `fn` succeed the first argument is `null` and the second argument 
	is an array of the arguments passing in all the callbacks.

	## Examples:

		awaitAll(loadImg, [['img1.png'], ['img2.png']], function(err, results){
			if (err) throw err;

			forEach(results, function(img){
				// do somthing with img.
			})
		});
	*/
	function awaitAll(fn, allArgs, callback, thisArg){
		thisArg = thisArg || null;
		var results = [];
		var counter = allArgs.length;
		var hasErrored = false;

		forEach(allArgs, function(args, i){
			fn.apply(thisArg, args.concat([function(err){
				if (hasErrored) return;

				if (err){
					hasErrored = true;
					return callback.call(thisArg, err);
				}

				results[i] = [].slice.call(arguments, 1);
				
				counter--;

				if (counter === 0) callback.call(thisArg, null, results);
			}]));
		});
	}

	/**
	# Function: getBoundingBox

	Gets the bounding box of two images.

	## Parameters:

	*	img1 - (HTMLImgElement) - An image element.
	*	img2 - (HTMLImgElement) - An image element.

	## Returns:

	*	(Object) - An object with a width and height property.
	*/
	function getBoundingBox(img1, img2){
		return {
			width: Math.max(img1.width, img2.width),
			height: Math.max(img1.height, img2.height)
		};
	}

	/**
	# Function: isCanvasSupported

	Checks if the canvas element is supported.

	## Returns:

	*	(Boolean) - Is `true` when canvas is supported.
	*/
	function isCanvasSupported(){
		var elem = document.createElement('canvas');
		return !!(elem.getContext && elem.getContext('2d'));
	}

	/**
	## Function: linear

	A timing function corisponding to the 'linear' CSS value.
	*/
	function linear(t){
		return t;
	}
	
	/**
	# Function: ease

	A timing function corisponding to the 'ease' CSS value.
	*/
	var ease = getCubicBezierTimingFunction(0.25, 0.0, 0.25, 1);

	/**
	# Function: easeIn

	A timing function corisponding to the 'ease-in' CSS value.
	*/
	var easeIn = getCubicBezierTimingFunction(0.42, 0, 1, 1);

	/**
	# Function: easeOut

	A timing function corisponding to the 'ease-out' CSS value.
	*/
	var easeOut = getCubicBezierTimingFunction(0, 0, 0.58, 1);

	/**
	# Function: easeInOut

	A timing function corisponding to the 'ease-in-out' CSS value.
	*/
	var easeInOut = getCubicBezierTimingFunction(0.42, 0, 0.58, 1);

	/**
	# Function: cubicBezier

	Given valid parameters returns a cubic bezier timing function, otherwise 
	the `stepStart` timing function.

	## Parameters:

	*	x1 - (Number) - The x axis value of control point 1.
	*	y1 - (Number) - The y axis value of control point 1.
	*	x2 - (Number) - The x axis value of control point 2.
	*	y2 - (Number) - The y axis value of control point 2.

	## Function:

	*	(Function) - A timing function.
	*/
	function cubicBezier(x1, y1, x2, y2){
		if (x1 < 0 || x1 > 1 || x2 < 0 || x2 > 1){
			console.warn('The x components of a cubic bezier must be in the range [0, 1].');
			return stepStart;
		}

		return getCubicBezierTimingFunction(x1, y2, x2, y2);
	}

	/**
	# Function: stepStart

	A timing function corisponding to the 'step-start' css value.
	*/
	var stepStart = steps(1, 'start');

	/**
	# Function: stepEnd

	A timing function corisponding to the 'step-end' css value.
	*/
	var stepEnd = steps(1, 'end');

	/**
	# Function: steps

	Gets a step timing function.

	## Parameters:
	
	*	intervals - (Number) - The number of steps.
	*	position - (String) - The string "start" or "end".

	## Returns:

	*	(Function) - A timing function.
	*/
	function steps(intervals, position){
		if (intervals < 1){
			console.warn('The intervals component of steps must be greater that 0.');
			return stepStart;
		}

		var isEndPosition = (position !== 'start');

		return getStepsTimingFunction(intervals, isEndPosition);
	}

	/**
	# Function: getStepsTimingFunction

	Gets a step timing function.

	A step timing function is a type of timing function that divides the input 
	time into a specified number of intervals that are equal in length.

	## Parameters:

	*	steps - (Number) - A positive interger for the number intervals.
	*	isEndPosition - (Boolean) - Should be `false` if the position component 
		of the step function is 'start'.
	## Returns:

	*	(Function) - A timing function.
	*/
	function getStepsTimingFunction(steps, isEndPosition){
		return function(inputProgress, before){
			var currentStep = Math.floor(inputProgress * steps)

			if (!isEndPosition) currentStep++;

			if (before && (inputProgress * steps) % 1 === 0){
				currentStep--;
			}

			if (inputProgress >= 0 && currentStep < 0) currentStep = 0;
			if (inputProgress <= 1 && currentStep > steps) currentStep = steps;

			return currentStep / steps;
		}
	}

	/**
	# Function: getTimingFunction

	Gets a timing function passed on a CSS transition timing function value.

	## Parameters:

	*	input - (String) - The CSS single timing function value.

	## Returns:

	*	(Function) - A timing function.
	*/
	function getTimingFunction(input){
		switch (input){
			case 'linear':
				return linear;

			case 'ease':
				return ease;

			case 'ease-in':
				return easeIn;

			case 'ease-out':
				return easeOut;

			case 'ease-in-out':
				return easeInOut;

			case 'step-start':
				return stepStart;

			case 'step-end':
				return stepEnd;
		}

		// cubid-bezier(<number>, <number>, <number>, <number>)
		var re1 = /^cubic-bezier\((?:\s*(([\+\-]?(?:\d+\.\d+|\.\d+|\d+)(?:[eE][\+\-]?\d+)?))\s*,){3}\s*(([\+\-]?(?:\d+\.\d+|\.\d+|\d+)(?:[eE][\+\-]?\d+)?))\s*\)$/;

		var result = re1.exec(input);

		if (result !== null){
			return cubicBezier(
				parseFloat(result[1]),
				parseFloat(result[2]),
				parseFloat(result[3]),
				parseFloat(result[4])
			);
		}

		// steps(<integer>[, start|end])
		var re2 = /^steps\(([\+\-]?\d+)(?:\s*,\s*(start|end))?\s*\)$/;

		result = re2.exec(input);

		if (result !== null){
			return steps(parseInt(result[1], 10), result[2] || '');
		}

	 	return stepStart;
	}

	/**
	# Function: requestAnimationFrame

	Polyfill for the requestAnimationFrmae function.
	*/
	var requestAnimationFrame =
		window.requestAnimationFrame ||
		window.mozRequestAnimationFrame ||
		window.webkitRequestAnimationFrame ||
		window.msRequestAnimationFrame ||
		function(fn){
			return setTimeout(fn, 40);
		};

	/**
	# Function: cancelAnimationFrame

	Polyfill for the cancelAnimationFrame function.
	*/
	var cancelAnimationFrame =
		window.cancelAnimationFrame ||
		window.mozCancelAnimationFrame ||
		window.webkitCancelAnimationFrame ||
		window.msCancelRequestAnimationFrame ||
		function(id){
			clearTimeout(id);
		};

	/**
	# Class: Animation

	An object to animate values from a starting value to an ending value.

	## Parameters:

	*	duration - (milliseconds) - The duration of the animation.
	*	properties - (Object<Array<Number, Number>>) - A map of values to animate.
	*	callback - (Function) - A function to call each frame of the animation.
	*	easeFn - (Function) - Optional. A function to provide easing.
	*	delay - (Number) - Optional. The number of milliseconds to delay the 
		animation.

	## Properties

	An object mapping a reference to an array with a starting value and an 
	ending value.

		{
			key1: [0, 100],
			ney2: [50, 0]
		}

	## Callback

	The callback is called for each frame and is passed an object of the 
	calculated values of each property.

		{
			key1: 20,
			key2: 40
		}

	*/
	var Animation = function(duration, properties, callback, easeFn, delay){
		this._duration = duration;
		this._properties = properties;
		this._callback = callback;
		this._easeFn = easeFn || null;
		this._delay = Math.max(delay || 0, 0);
		this._stopped = false;
		this._requestId = null;
		this._startTime = null;
	};

	Animation.prototype = {
		/**
		## Function: _tick

		Private.

		The function thats is called for every frame of the animation. Calculates the
		progress through the animation and if the animation should complete.
		*/
		_tick: function(){
			if (this._stopped)  return;

			var duration = this._duration;
			var startTime = this._startTime;
			var properties = this._properties;

			var now = +(new Date());
			var progress = (now - (startTime)) / duration;

			progress = Math.min(1, progress);

			var before = false;

			if (progress < 0){
				progress = 0;
				before = true;
			}

			if (this._easeFn) progress = this._easeFn(progress, before);

			var state = {};

			for (var k in properties) if (properties.hasOwnProperty(k)){
				state[k] = this._getState(progress, properties[k]);
			}

			if (progress === 1){
				this._stopped = true;
			}

			if (this._stopped){
				this._requestId =  null;
			}
			else {
				this._requestId = requestAnimationFrame(bind(this._tick, this));
			}

			this._callback(state);
		},

		/**
		## Function: _getState

		Private.

		Calculates a property value for the current frame of the animation.

		### Parameters:

		*	percent - (Number) - A value from 0 to 1. The percent through the 
			animation.
		*	values - (Array<Number, Number>) - The starting value and ending value.

		### Returns:

		*	(Number) - The value for the current frame.
		*/
		_getState: function(percent, values){
			var start = values[0];
			var end = values[1];
			return ((end - start) * percent) + start;
		},

		/**
		## Function: start

		Starts the animation.
		*/
		start: function(){
			this._startTime = (+(new Date())) + this._delay;
			this._stopped = false;

			this._requestId = requestAnimationFrame(bind(this._tick, this));
		},

		/**
		## Function: stop

		Stops the animation prematurely.
		*/
		stop: function(){
			this._stopped = true;

			if (this._requestId !== null){
				cancelAnimationFrame(this._requestId);
				this._requestId = null;
			}
		}
	};

	/**
	# Class: BasicTransition

	A simple transition that just swaps the image without any effects.

	## Parameters:

	*	target - (HTMLImgElement) - The img element in the DOM to affect.
	*	img1 - (HTMLImgElement) - The first image.
	*	img2 - (HTMLImgElement) - The second image.
	*/
	var BasicTransition = function(target, img1, img2){
		this._target = target;
		this._img1 = img1;
		this._img2 = img2;
	};

	BasicTransition.prototype = {
		/**
		## Function: stop

		Stops the transition.
		*/
		stop: function(){
			//noop
		},

		/**
		## Function: gotoStart

		Transitions to the starting state, showing the first image.
		*/
		gotoStart: function(){
			var target = this._target;
			var img1 = this._img1;
			target.src = img1.src;
			target.setAttribute('width', img1.width);
			target.setAttribute('height', img1.height);
		},

		/**
		## Function: gotoEnd

		Transitions to the ending state, showing the second image.
		*/
		gotoEnd: function(){
			var target = this._target;
			var img2 = this._img2;
			target.src = img2.src;
			target.setAttribute('width', img2.width);
			target.setAttribute('height', img2.height);
		},

		/**
		## Function: setDelay

		Set the delay on the animation in milliseconds.

		### Parameters:

		*	value - (Number) - The delay in milliseconds.
		*/
		setDelay: function(){
			//noop
		},

		/**
		## Function: setDuration

		Set the duration on the animation in milliseconds.

		### Parameters:

		*	value - (Number) - The duration in milliseconds.
		*/
		setDuration: function(){
			//noop
		},

		/**
		## Function: setDuration

		Set the timing function to use.

		### Parameters:

		*	value - (Function) - The timing function to provide easing.
		*/
		setTimeingFunction: function(){
			//noop
		}
	};

	/**
	# Class: CanvasTransition

	Uses a canvas element to transition bettween imgages, optional with a fade 
	effect.

	## Parameters:

	*	target - (HTMLImgElement) - The img element in the DOM to affect.
	*	img1 - (HTMLImgElement) - The first image.
	*	img2 - (HTMLImgElement) - The second image.
	*	useFade - (Boolean) - Should be `true` if a fade effect is desired.
	*/
	var CanvasTransition = function(target, img1, img2, useFade){
		this._target = target;
		this._img1 = img1;
		this._img2 = img2;
		this._useFade = useFade;

		var boundingBox = this._boundingBox = getBoundingBox(img1, img2);

		var canvas = this._canvas = document.createElement('canvas');
		canvas.width = boundingBox.width;
		canvas.height = boundingBox.height;

		var ctx = this._ctx = canvas.getContext('2d');

		ctx.drawImage(img1, (boundingBox.width - img1.width) / 2, (boundingBox.height - img1.height) / 2);

		target.src = canvas.toDataURL();

		target.setAttribute('width', boundingBox.width);
		target.setAttribute('height', boundingBox.height);

		this._img1Alpha = 1;
		this._img2Alpha = 0;
		this._animation = null;
		this._animationDuration = 200;
		this._animationDelay = 0;
		this._animationTimingFunction = easeInOut;
		this._imgLoaded = true;
	};

	CanvasTransition.prototype = {
		/**
		## Function: stop

		Stops the transition.
		*/
		stop: function(){
			var animation = this._animation;
			if (animation !== null) animation.stop();
			this._animation = null;
		},
		
		/**
		## Function: gotoStart

		Transitions to the starting state, showing the first image.
		*/
		gotoStart: function(){
			var target = this._target;
			var boundingBox = this._boundingBox;
			var canvas = this._canvas;
			var ctx = this._ctx;
			var img1 = this._img1;
			var img2 = this._img2;

			if (!this._useFade){
				ctx.clearRect(0, 0, boundingBox.width, boundingBox.height);

				ctx.drawImage(img1, (boundingBox.width - img1.width) / 2, (boundingBox.height - img1.height) / 2);
				
				target.src = canvas.toDataURL();
				return;
			}

			this._animation = new Animation(
				this._animationDuration, 
				{
					img1: [this._img1Alpha, 1],
					img2: [this._img2Alpha, 0]
				}, 
				bind(this._onAnimationFrame, this),
				this._animationTimingFunction,
				this._animationDelay
			);

			this._animation.start();
		},
		
		/**
		## Function: gotoEnd

		Transitions to the ending state, showing the second image.
		*/
		gotoEnd: function(){
			var target = this._target;
			var boundingBox = this._boundingBox;
			var canvas = this._canvas;
			var ctx = this._ctx;
			var img1 = this._img1;
			var img2 = this._img2;

			if (!this._useFade){
				ctx.clearRect(0, 0, boundingBox.width, boundingBox.height);

				ctx.drawImage(img2, (boundingBox.width - img2.width) / 2, (boundingBox.height - img2.height) / 2);
				
				target.src = canvas.toDataURL();
				return;
			}

			this._animation = new Animation(
				this._animationDuration, 
				{
					img1: [this._img1Alpha, 0],
					img2: [this._img2Alpha, 1]
				}, 
				bind(this._onAnimationFrame, this),
				this._animationTimingFunction,
				this._animationDelay
			);

			this._animation.start();
		},

		/**
		## Function: setDelay

		Set the delay on the animation in milliseconds.

		### Parameters:

		*	value - (Number) - The delay in milliseconds.
		*/
		setDelay: function(value){
			this._animationDelay = value;
		},

		/**
		## Function: setDuration

		Set the duration on the animation in milliseconds.

		### Parameters:

		*	value - (Number) - The duration in milliseconds.
		*/
		setDuration: function(value){
			this._animationDuration = value;
		},

		/**
		## Function: setDuration

		Set the timing function to use.

		### Parameters:

		*	value - (Function) - The timing function to provide easing.
		*/
		setTimeingFunction: function(value){
			this._animationTimingFunction = value;
		},

		/**
		## Function: _onAnimationFrame

		Private.

		Updates the target element every frame of the fade effect.

		### Parameters:

		*	state - (Object) - The animation state of the alpha values of the 
			two images.
		*/
		_onAnimationFrame: function(state){
			var img1Alpha = state.img1;
			var img2Alpha = state.img2;

			// If delaying or using a steped timing function avoid unnessasary 
			// DOM touching.
			if (img1Alpha === this._img1Alpha && img2Alpha === this._img2Alpha){
				return;
			}

			if (!this._imgLoaded && img1Alpha !== 1 && img1Alpha !== 0 && img2Alpha !== 0 && img2Alpha !== 1){
				return;
			}

			this._img1Alpha = img1Alpha;
			this._img2Alpha = img2Alpha;

			var target = this._target;
			var boundingBox = this._boundingBox;
			var canvas = this._canvas;
			var ctx = this._ctx;
			var img1 = this._img1;
			var img2 = this._img2;

			ctx.clearRect(0, 0, boundingBox.width, boundingBox.height);

			ctx.globalAlpha = img1Alpha;

			ctx.drawImage(img1, (boundingBox.width - img1.width) / 2, (boundingBox.height - img1.height) / 2);

			ctx.globalAlpha = img2Alpha;

			ctx.drawImage(img2, (boundingBox.width - img2.width) / 2, (boundingBox.height - img2.height) / 2);

			this._imgLoaded = false;

			target.src = canvas.toDataURL();

			target.onload = bind(function(){
				this._imgLoaded = true;
			}, this);
		}
	};

	/**
	# Function: getTransition

	Picks a transition to use.

	## Parameters:

	*	target - (HTMLImgElement) - The img element in the DOM to affect.
	*	img1 - (HTMLImgElement) - The first image.
	*	img2 - (HTMLImgElement) - The second image.
	*	useFade - (Boolean) - Should be `true` if a fade effect is desired.

	## Returns:

	*	(CanvasTransition|BasicTransition) - A transition object.
	*/
	function getTransition(target, img1, img2, useFade){
		if (!isCanvasSupported()){
			return new BasicTransition(target, img1, img2);
		}
		return new CanvasTransition(target, img1, img2, useFade);
	}

	/**
	# Function: ThumbnailHover

	Object to manage the state of each thumbnail.

	## Parameters:

	*	element - (HTMLImgElement) - The product image.
	*	src1 - (String) - The url to the first image.
	*	src2 - (String) - The url to the second image.
	*	useFade - (Boolean) - When `true` a fade effect may be used to 
		transition bettween the two images.
	*/
	var ThumbnailHover = function(element, src1, src2, useFade, opacityTransition){
		this._element = element;

		awaitAll(loadImg, [[src1], [src2]], function(err, results){
			if (err) return;

			var img1 = results[0][0];
			var img2 = results[1][0];

			var t = this._transition = getTransition(element, img1, img2, useFade);
			t.setDuration(opacityTransition.duration);
			t.setDelay(opacityTransition.delay);
			t.setTimeingFunction(getTimingFunction(opacityTransition.timingFunction));

			this._setupEventListeners();
		}, this);
	};

	ThumbnailHover.prototype = {
		/**
		## Function: _setupEventListeners

		Private.

		Sets up the event listeners on the element for the mouse entering or 
		leaving.
		*/
		_setupEventListeners: function(){
			var element = this._element;
			addEvent(element, 'mouseenter', bind(this._onMouseEnter, this));
			addEvent(element, 'mouseleave', bind(this._onMouseLeave, this));
		},

		/**
		## Function: _onMouseEnter

		Private.

		Event handler for the mouse entering the element.
		*/
		_onMouseEnter: function(event){
			this._showImg2();
		},

		/**
		## Function: _onMouseLeave

		Private.

		Event handler for the mouse leaves the element.
		*/
		_onMouseLeave: function(event){
			this._showImg1();
		},

		/**
		## Function: _showImg2

		Private.

		Shows image two.
		*/
		_showImg2: function(){
			this._transition.stop();
			this._transition.gotoEnd();
		},

		/**
		## Function: _showImg1

		Private.

		Shows image one.
		*/
		_showImg1: function(){
			this._transition.stop();
			this._transition.gotoStart();
		},
	};

	/**
	# Function: getFromComputedStyle

	Gets a property from a CSSStyleDeclaration acounting for vendor prefixes.

	## Parameters:

	*	computedStyles - (CSSStyleDeclaration) - the computed styles object.
	*	property - (String) - The unprefixed name of the property to get.

	## Returns:

	*	(String) - the property value or an empty string.
	*/
	function getFromComputedStyle(computedStyles, property){
		var prefixes = ['-webkit-', '-moz-', '-o-', ''];

		var value;
		var i = prefixes.length - 1;

		while (!value && i>=0){
			value = computedStyles[prefixes[i] + property];
			i--;
		}

		return value || '';
	}

	/**
	# Function: parseCSSTime

	Turns a CSS time value into a number or milliseconds.

	## Parameters:

	*	input - (String) - The CSS time to parse.

	## Returns:

	*	(Number) - The pased time value converted to a number and milliseconds.

	## Examples:

		parseCSSTime('2') === 2000;
		parseCSSTime('2s') === 2000;
		parseCSSTime('2ms') === 2;
		parseCSSTime('.2s') === 200;
	*/
	function parseCSSTime(input){
		var value = parseFloat(input);
		if (input.slice(-2) !== 'ms') value = value * 1000;
		return value;
	};

	/**
	# Function: getOpacityTransitionStyle

	Gets the CSS transition values for the opacity property.

	## Parameters:

	*	element - (HTMLElement) - The element to get the style of.

	## Returns:

	*	(Object) - An object describing the transition. See Transition.

	## Transition:

	An Object with the properties:

	*	duration - (Number) - A number of milliseconds.
	*	delay - (Number) - A number of milliseconds.
	*	timingFunction - (String) - The timing function.
	*/
	function getOpacityTransitionStyle(element){
		var style = {
			duration: 0,
			delay: 0,
			timingFunction: 'ease'
		};

		if (!window.getComputedStyle) return style;

		var computedStyles = getComputedStyle(element);

		var properties = getFromComputedStyle(computedStyles, 'transition-property').split(',');
		var durations = getFromComputedStyle(computedStyles, 'transition-duration').split(',');
		var timingFunctions = getFromComputedStyle(computedStyles, 'transition-timing-function').split(',');
		var delays = getFromComputedStyle(computedStyles, 'transition-delay').split(',');

		// Find the index of the last opacity property.
		var index = -1;
		for (var i=properties.length - 1; i>=0; i--){
			if (properties[i] === 'opacity' || properties[i] === 'all'){
				index = i;
				break;
			}
		}

		if (index === -1) return style;

		// Fix  any cubic-bezier or steps items in timingFunctions.
		var a = [];
		var i=0;
		var p=0;
		var len = timingFunctions.length;
		while (i < len){
			if (/^(?:cubic-bezier|steps)\(/.test(timingFunctions[i])){
				var j = i;
				while (j<len && !/\)/.test(timingFunctions[j])) j++;
				a[p] = timingFunctions.slice(i, j + 1).join(',');
				i = j + 1;
			}
			else {
				a[p] = timingFunctions[i];
				i++;
			}
		}
		timingFunctions = a;

		// Repeat values to fill out any missmatched array lengths, upto the 
		// index we care about.
		forEach([durations, timingFunctions, delays], function(ary){
			var s = ary.length;
			var i = 0;
			while (s < index) {
				ary[s] = ary[i];
				s++;
				i++;
			}
		});

		style.duration = parseCSSTime(durations[index]);
		style.delay = parseCSSTime(delays[index]);
		style.timingFunction = timingFunctions[index];

		return style;
	}


	/**
	# Function: onReady

	Sets up the thumbnail hover.
	*/
	function onReady(){
		var elements = document.querySelectorAll("a[data-on-hover-show]");

		forEach(elements, function(element){
			var useFade = element.getAttribute('data-on-hover-fade') === 'true';
			var img = element.getElementsByTagName('img')[0] || null;

			var opacityTransition = getOpacityTransitionStyle(img);
			
			if (img === null) return;

			var url1 = img.src;
			var url2 = element.getAttribute("data-on-hover-show");

			var thumbnailHover = new ThumbnailHover(img, url1, url2, useFade, opacityTransition);
		});
	}

	/**
	# Function: contentLoaded

	Executes a callback once the the DOM has become accessable.

	## Credit:

	*	Diego Perini (diego.perini at gmail.com) 
		(https://github.com/dperini/ContentLoaded)
	*/
	function contentLoaded(win, fn){
		var done = false;
		var top = true;

		var doc = win.document;
		var root = doc.documentElement;

		var add = doc.addEventListener ? 'addEventListener' : 'attachEvent';
		var rem = doc.addEventListener ? 'removeEventListener' : 'detachEvent';
		var pre = doc.addEventListener ? '' : 'on';

		var init = function(e){
			if (e.type == 'readystatechange' && doc.readyState != 'complete'){
				return;
			}

			(e.type == 'load' ? win : doc)[rem](pre + e.type, init, false);

			if (!done && (done = true)){
				fn.call(win, e.type || e);
			}
		};

		var poll = function(){
			try {
				root.doScroll('left');
			} 
			catch(e) {
				setTimeout(poll, 50); return;
			}
			init('poll');
		};

		if (doc.readyState == 'complete'){
			fn.call(win, 'lazy');
		}
		else {
			if (doc.createEventObject && root.doScroll) {
				try {
					top = !win.frameElement;
				} 
				catch (e){}

				if (top){
					poll();
				}
			}
			doc[add](pre + 'DOMContentLoaded', init, false);
			doc[add](pre + 'readystatechange', init, false);
			win[add](pre + 'load', init, false);
		}
	}

	contentLoaded(window, onReady);
})(window, document);