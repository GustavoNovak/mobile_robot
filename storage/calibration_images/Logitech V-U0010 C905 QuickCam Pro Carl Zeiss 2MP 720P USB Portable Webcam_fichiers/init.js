//Initalise the lightbox plugin on elements with the rel attribute containing the string 'lightbox'.
(function($){			
	$(function(){
		var settings = {}
		
		if (lightbox_mobile_check){
			settings.margin = 30;
			settings.top = 15;
		}

		var imageElements = $('*[rel*="lightbox"]');
		imageElements.sort(sortJQOject);
		imageElements.lightBox(settings);
	});

	function sortJQOject(a, b) {
		if ($(a).attr("id")>$(b).attr("id")) { return 1; }
		if ($(a).attr("id")<$(b).attr("id")) { return -1; }
		return 0;
	}

})(ekm.$);