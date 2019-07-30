// Intentially missing var keyword. 
_ekmpinpoint = window._ekmpinpoint || {};

_ekmpinpoint.getStats = function(site){
	var srv = "//ekmpinpoint.ekmsecure.com/harvest/collect.asp";

	// collect all the data to encode into the url.
	var data = {
		site: site,
		ref: document.referrer,
		loc: document.location,
		res: screen.width + "x" + screen.height,
		title: document.title,
		rnd: +(new Date())
	};

	// encode the data into the url.
	var a = [];

	for (var k in data) if (data.hasOwnProperty(k)){
		a[a.length] = encodeURIComponent(k) + '=' + encodeURIComponent(data[k]);
	}

	var url = srv + '?' + a.join('&');

	// create the image element.
	var img = document.createElement('img');
	img.setAttribute('width', '1');
	img.setAttribute('height', '1');
	img.setAttribute('border', '0');
	img.setAttribute('alt', '');
	img.style.position = 'absolute';
	img.style.left = '-9px';
	img.style.top = '-9px';
	img.src = url;

	// Inject element into DOM as a sibling of this scripts' element.
	var scripts = document.getElementsByTagName('script');
	var script = scripts[scripts.length - 1];
	script.parentNode.appendChild(img);
};