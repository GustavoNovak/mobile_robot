(function(window){
	if (!window.sessionStorage) return;
	if (!window.JSON) return;

	var SESSION_KEY = '_ekm_softcart_history_tracker';
	var TRACKING_LIMIT = 9;
	var EXPIRY_LIMIT =  3.6e+6 //milliseconds

	var getHistory = function(){
		var history = sessionStorage.getItem(SESSION_KEY);
		return (history && JSON.parse(history)) || [];
	};

	var setHistory = function(history){
		sessionStorage.setItem(SESSION_KEY, JSON.stringify(history));
	};

	var hasExpired = function(history, now){
		if (history.length === 0) return false;

		var entry = history[history.length - 1];
		var then = new Date(entry.date);

		return (now - then) > EXPIRY_LIMIT;
	};

	var isBack = function(history, here){
		return history.length > 1 && history[history.length -2].href === here;
	};

	var isRefresh = function(history, here){
		return history.length > 0 && history[history.length -1].href === here;
	};

	var dateToString = function(date){
		if (date.toISOString) return date.toISOString();
		return date.toString();
	};

	var pushEntry = function(history, here, now){
		history[history.length] = {
			href: here,
			date: dateToString(now)
		};
	};

	var prune = function(history){
		if (history.length > TRACKING_LIMIT){
			history = history.slice(history.length - TRACKING_LIMIT);
		}

		return history;
	};

	var createEvent = function(name, detail){
		if ('CustomEvent' in window && typeof CustomEvent === 'function'){
			return new CustomEvent(name, {detail: detail});
		}

		if (window.document.createEvent){
			var event = window.document.createEvent('Event');

			event.initEvent(name, true, false);
		}
		else if (window.document.createEventObject){
			var event = window.document.createEventObject();
			event['type'] = name;
		}
		else {
			throw new Error('Cannot create event');
		}

		event.detail = detail;

		return event;
	};

	var dispatchEvent = function(thing, event){
		thing.dispatchEvent(event);
	};

	var checkHistory = function(){
		var history = getHistory();

		var here = window.location.href;
		var now = new Date();

		if (hasExpired(history, now)) history = [];

		var value = null;

		if (isBack(history, here)){
			value = 'back';
			history.pop();
		}
		else {
			if (isRefresh(history, here)){
				value = 'refresh';
				history.pop();
			}

			pushEntry(history, here, now);
		}

		history = prune(history);

		setHistory(history);

		return value;
	};

	var go = function(){
		var detail = checkHistory();

		if (detail !== null){
			setTimeout(function(){
				// Should fire the event after all scripts have had a chance to 
				// setup a listener.
				dispatchEvent(window, createEvent('history', detail));
			}, 5);
		}
	};

	go();
})(window);