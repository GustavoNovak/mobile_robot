var $j = jQuery;

// Mobile Header Accordion
(function(){
// Close all open accordions.
function closeAccordions( accordion ) {
	var accoridon = accordion;
	var contents = [].slice.call( document.querySelectorAll( "[data-" + dataPanel + "]" ), 0 );
	contents.forEach( function( content ){
		content.classList.add( classHidden );
	});
}

// Open accordion.
function openAccordion( element ) {
	element.classList.remove( classHidden );
}

// Make clicked heading active.
function headingActive( heading ) {
	heading.classList.add( classActive );
}

// Make all headings in-active.
function headingInactive() {
	var headings = [].slice.call( document.querySelectorAll( "[data-" + dataHeading + "]" ), 0 );
	headings.forEach( function( heading ){
		heading.classList.remove( classActive );
	});
}

var accordions = [].slice.call( document.querySelectorAll( ".header-section" ), 0 ),
	dataHeading = "heading",
	dataPanel   = "panel",
	classHidden = "header-btn-hidden",
	classActive = "header-btn-active";


if ( accordions ) {
	accordions.forEach( function( accordion ){
		var headings = [].slice.call( document.querySelectorAll( "[data-" + dataHeading + "]" ), 0 );
		headings.forEach( function( heading ) {
			heading.addEventListener( "click", function( event ) {
				event.preventDefault();
				
				var content = this.getAttribute("data-" + dataHeading + ""),
					panel   = document.querySelector( "[data-" + dataPanel + "='" + content + "']" );

				var shouldOpen = !heading.classList.contains( classActive );

				closeAccordions( accordion );
				headingInactive( this );

				if ( shouldOpen ) {
					openAccordion( panel );
					headingActive( this );
				}				
			});
		});
	});
}
})();



//Slide Out Header jQuery
$j('.slide-btn').click(function() {
    $j(this).next('.slide-dd').fadeIn(150);
    $j("input:text:visible:first").focus();
});

$j('.close-btn').click(function() {
    $j('.slide-dd').fadeOut(150);
});



// Click Navigation Dropdown
$j('.nav-d ul li:has(ul)').addClass('droppable nav-chevron-down');

$j(".droppable").click(function(e){
	e.preventDefault();
	e.stopPropagation();
	$j('.droppable.nav-active').not(this).removeClass('nav-active');
	$j('.droppable.nav-chevron-up').not(this).removeClass('nav-chevron-up').addClass('nav-chevron-down');
	$j(this).toggleClass('nav-active nav-chevron-down nav-chevron-up');
});

$j(".droppable .mega-menu").click(function(e){
	e.stopPropagation();
});

$j(document).click(function(e) {
  if( e.target.class != '.droppable') {
    $j('.droppable').removeClass('nav-active');
	$j('.droppable.nav-chevron-up').not(this).removeClass('nav-chevron-up').addClass('nav-chevron-down');
  }
});


	
// Filter Accordion Toggle
$j('.filter-item-heading').on('click', function(){
	$j(this).siblings('.filter-item-list').stop().slideToggle(150);
	$j(this).toggleClass('filter-minus');
	$j(this).toggleClass('filter-plus');
});



// Mobile Navigation Drawer
$j(function() {                       
	$j(".nav-btn").click(function() {  
		$j(".nav-drawer-m").addClass("nav-active-m");  
		$j(".nav-overlay-m, .nav-close-btn-m").fadeIn();
		$j("body").addClass("nav-scroll-m");
	});
	
	$j(".nav-close-btn-m").click(function() {  
		$j(".nav-drawer-m").removeClass("nav-active-m");
		$j(".nav-overlay-m, .nav-close-btn-m").fadeOut();
		$j("body").removeClass("nav-scroll-m");
	});
});

$j(document).mouseup(function(e) {
	var container = $j(".nav-drawer-m");

	if (!container.is(e.target) && container.has(e.target).length === 0) 
    {
		$j(".nav-drawer-m").removeClass("nav-active-m");
		$j(".nav-overlay-m, .nav-close-btn-m").fadeOut();
		$j("body").removeClass("nav-scroll-m");
    }
});



// Mobile Filter Dropdown
$j('.filter-section-btn-m').addClass('filter-chevron-down-m');

$j('.filter-section-btn-m').click(function() {
	$j(this).next().toggle();
	$j(this).toggleClass('filter-btn-active-m filter-chevron-up-m');
	$j('.filter-section-btn-m').not(this).removeClass('filter-btn-active-m filter-chevron-up-m');
});

$j(document).mousedown(function (e){
    var container = $j(".sortby-content-m, .filter-content-m");
    var btn = $j(".sortby-content-m, .filter-content-m, .filter-section-btn-m");
    if (!container.is(e.target)
        && container.has(e.target).length == 0) 
    {
        container.fadeOut(100);
    }
    
    if (!btn.is(e.target)
        && btn.has(e.target).length == 0) 
    {
        btn.removeClass('filter-btn-active-m filter-chevron-up-m');
    }
});



// Global Accordion
$j('.prod-acdn-content:first').show();
$j('.prod-acdn-header:first').addClass('global-acdn-active');

$j('.global-acdn-header').click(function(e) {
  	e.preventDefault();
  	
  
    
    $j(this).parent().parent().find('.global-acdn-content').slideUp(250);
      
    if ($j(this).next().css('display') === 'none'){
      $j(this).next().slideDown(250);
    }
      
    else {
      $j(this).next().slideUp(250);
    }
  
    if ($j(this).hasClass('global-acdn-active')){
      $j(this).removeClass('global-acdn-active');	
    } 
  
    else {
      $j('.global-acdn-header').removeClass('global-acdn-active');
      $j(this).addClass('global-acdn-active');	
    }
});	



// Product Accordion Scroll To 
$j('.prod-acdn-header').click(function(e) {
  	e.preventDefault();
  	
  	var headerBar = $j('.header-section').outerHeight(),
	    header = headerBar;
    
    $j('html, body').animate({
    	scrollTop: $j('.prod-acdn-header').offset().top - header - 20 
    }, '100');
});	



//Mobile Banner Zoom
(function(){
	var banner = document.querySelector( ".js-banner" );
	
	if ( banner ) {
		var isCarousel = banner.querySelector( ".ekm-banner" );
	
		if ( isCarousel ) {
			banner.classList.add( "banner-is-carousel" );
		} 
		
		else {
			banner.classList.add( "banner-is-static" );
		}
	}
})();

$j(window).load(function() {
	if ($j('.banner-is-static').length !== 0){
		var width = $j(document).width();
		if (!$j('.banner-is-static img')[0]) return;
		var scaleHeight = $j('.banner-is-static img')[0].getBoundingClientRect().height
	
		if (width < 780) {
			$j('.banner-is-static').css({'height': scaleHeight});
			$j('.banner-is-static').height(function (index, height) {return (height - 18);});
		}
			
		$j(window).resize(function() {
			scaleHeight = $j('.banner-is-static img')[0].getBoundingClientRect().height
			if ($j(window).width() >= 781) {
				$j(".banner-is-static").height('auto');
			}

			if ($j(window).width() < 780) {
				$j('.banner-is-static').css({'height': scaleHeight});
				$j('.banner-is-static').height(function (index, height) {return (height - 18);});
			}
		});
		
	}
});



// Product Page Thumbnail Show/Hide
(function(){
	function changeEventHandler( event ) {
	    imageWrappers.forEach( function( imageWrapper ) {
	    	var href = imageWrapper.querySelector( "a" ).getAttribute("href");

	    	if ( href === "#" ) {
	    		imageWrapper.parentNode.classList.add("thumbnail-no-image");
	    	} else {
	    		imageWrapper.parentNode.classList.remove("thumbnail-no-image");
	    	}
	    });
	}
	
	var	imageWrappers = [].slice.call( document.querySelectorAll( ".thumbnail-image" ), 0);

})();



// Sticky Header Margin
$j(window).load(function() {
	var headerBar = $j('.header-section').outerHeight(),
	    header = headerBar;
	
	$j('.main-content').css({
		'margin-top': header
	});
});

$j(window).resize(function() {
	var headerBar = $j('.header-section').outerHeight(),
	    header = headerBar;
	
	$j('.main-content').css({
		'margin-top': header
	});
});



//Fix To Colour Swatch/Dropdowns Select Arrow 
$j('.ekm-productoptions-dropdown-option').parent().addClass('options-select-dropdown');



// Product Name Overflow
var productNames = [].slice.call( document.querySelectorAll( ".prod-item-name-ellipsis" ), 0 );

if ( productNames ) {
	productNames.forEach( function( element ){
		var wrapperWidth = element.offsetWidth,
			nameWidth    = element.querySelector( "a" ).offsetWidth;
		if ( nameWidth > wrapperWidth ) {
			element.parentNode.classList.add( "prod-item-name-overflow" );
		}
	});
}