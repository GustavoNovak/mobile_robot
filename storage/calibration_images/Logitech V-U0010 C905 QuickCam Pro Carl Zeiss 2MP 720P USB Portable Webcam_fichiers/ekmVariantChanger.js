if (!window.CSS || !window.CSS.escape) document.write('<script src="https://cdn.jsdelivr.net/npm/css.escape@1.5.1/css.escape.min.js"></script>');

function Variants_ProductsOptions(event){

	if(window.ekmHideOutOfStockVariants != undefined && window.ekmHideOutOfStockVariants == true){
		buildAvaliableOptions();
		displaySelectors(event);
	}

	var doIEShowHideOptions = false;
	var functionArrayList = {};
	var selectedVariantProperties = {};

	if (!ekmProductVariantData) return;
	
	if (typeof Object.assign != 'function') {
	  // Must be writable: true, enumerable: false, configurable: true
	  Object.defineProperty(Object, "assign", {
	    value: function assign(target, varArgs) { // .length of function is 2
	      'use strict';
	      if (target == null) { // TypeError if undefined or null
	        throw new TypeError('Cannot convert undefined or null to object');
	      }

	      var to = Object(target);

	      for (var index = 1; index < arguments.length; index++) {
	        var nextSource = arguments[index];

	        if (nextSource != null) { // Skip over if undefined or null
	          for (var nextKey in nextSource) {
	            // Avoid bugs when hasOwnProperty is shadowed
	            if (Object.prototype.hasOwnProperty.call(nextSource, nextKey)) {
	              to[nextKey] = nextSource[nextKey];
	            }
	          }
	        }
	      }
	      return to;
	    },
	    writable: true,
	    configurable: true
	  });
	}

	for (var i=0;i<ekmProductVariantData.length;i++){

		if (ekmProductVariantData[i]) {
			if (!ekmProductVariantData[i].items) return;

			for (var n=0;n<ekmProductVariantData[i].items.length;n++){

				var variant = ekmProductVariantData[i].items[n];

				for (var si=0; si<variant.selector.length; si++){
					if (variant.selector[si].type != 'DROPDOWN'){
						if (document.querySelector('input[name="variant_' + CSS.escape(variant.selector[si].option) + '"]').value == '') {
							setRadioVariantData(variant.selector[si], variant.selector[si].option); //set the hidden fields value here (page_parser)
						}
					}
				}
				
				if (n==0 || (variant.hasOwnProperty('selector') && ekm_checkSelected(variant.selector))) {

					if (variant.hasOwnProperty('properties')){
						
						if(Object.keys(selectedVariantProperties).length === 0) {
							Object.assign(selectedVariantProperties, variant.properties);
						} else {
							var variantPropertyNames = Object.keys(variant.properties);
							var variantPropertyNamesLength = variantPropertyNames.length;
							for(var j = 0; j < variantPropertyNamesLength; j++) {
								var variantPropertyName = variantPropertyNames[j];
								if(selectedVariantProperties.hasOwnProperty(variantPropertyName) &&
									selectedVariantProperties[variantPropertyName].elementId === variant.properties[variantPropertyName].elementId) {
									selectedVariantProperties[variantPropertyName] = Object.assign({}, variant.properties[variantPropertyName]);
								} else {
									selectedVariantProperties[variantPropertyName + "000"] = variant.properties[variantPropertyName];
								}
							}
						}

						for (var property in variant.properties){
							var variantProperty = variant.properties[property]

							if (variantProperty.hasOwnProperty('elementId')){
								var key = variantProperty.elementId;

								if (document.getElementById(key)){
									var elements = ekm_getElementsById(key);
									for (var x=0;x<elements.length;x++){
										if (variantProperty.hasOwnProperty('innerHTML')){

											elements[x].innerHTML = ekm_compositeInnerHtml(elements[x], variantProperty.innerHTML);
											showHideAttributes(variantProperty);										
										}

										if (variantProperty.hasOwnProperty('content')){
											elements[x].setAttribute('content', variantProperty.content);
										}

										if (variantProperty.hasOwnProperty('display')){
											elements[x].style.display = variantProperty.display;
										}

										if (variantProperty.hasOwnProperty('src')){
											if (variantProperty.src == "") {
												elements[x].src = "/ekmps/images/spacer.gif";
											}
											else {
												elements[x].src = variantProperty.src;
											}
										}

										if (variantProperty.hasOwnProperty('href')){
											elements[x].href = variantProperty.href;
										}


										if (variantProperty.hasOwnProperty('value')){
											elements[x].value = variantProperty.value;
										}

										if (variantProperty.hasOwnProperty('width')){
											elements[x].setAttribute('width', variantProperty.width);
										}

										if (variantProperty.hasOwnProperty('height')){
											elements[x].setAttribute('height', variantProperty.height);
										}

										for (var dataProperty in variantProperty){
											if (dataProperty.indexOf("data-") > -1){
												elements[x].setAttribute(dataProperty, variantProperty[dataProperty]);
											}
										}
									}
								}
							}

							if (variantProperty.hasOwnProperty('function')){
								if (typeof(variantProperty["function"]) == "function") {
									functionArrayList[property] = variantProperty["function"];
								}
							}
						}
					}
				}
			}
		}
	}

	for (var item in functionArrayList) {
		functionArrayList[item](selectedVariantProperties);
	}
}

function showHideAttributes (varProp) {
if (varProp.elementId.match(/_EKM_PRODUCTATRRIBUTE_.*?/) && varProp.elementId.match(/.*?_VALUE/)) {
		var wrapper = document.getElementById(varProp.elementId.replace('_VALUE', '_WRAPPER'));
		
		if (typeof wrapper !== 'undefined' && wrapper !== null) {
			wrapper.style.display = (varProp.innerHTML === '') ? 'none' : '';
		}
	}
}

function ekm_compositeInnerHtml (element, innerHtml) {
	var changeListener = element.getAttribute('data-change-listener');
	if (changeListener) {
		var fn = window[changeListener];
		if (typeof fn === 'function'){
			var args = [
			element
			]
			try {
				fn.apply(fn, args);
			}
			catch(ex){}
		}
	}

	if (innerHtml === "") {
		return "";
	}

	// These attributes apply to the out of stock message
	var outputStart = element.getAttribute('data-output-start') || "";
	var outputEnd = element.getAttribute('data-output-end') || "";
	var removeFormatting = element.getAttribute('data-remove-formatting') || "";
	if (removeFormatting === "false") {
		innerHtml = "<strong>" + innerHtml + "</strong><br />"
	}

	return outputStart + innerHtml + outputEnd;
}

ekmFunc_addEvent(window,'load',ekmFunc_setVariantPageLoad);
function ekmFunc_setVariantPageLoad() {

	var selectVariant = [];
	if (typeof(ekmVariantRedirect)!='undefined') {
		selectVariant = ekmVariantRedirect
	}
	else if (typeof(ekmProductinStock)!='undefined') {
		selectVariant = ekmProductinStock
	}
	else {

	}


	for (var i=0;i<selectVariant.length;i++){
		var item = selectVariant[i];
		if (item.hasOwnProperty('option') && item.hasOwnProperty('value')) {
			if (item.dataValue == '') {
				ekmFunc_setSelectedIndex(document.getElementById('variant_' + item.option),item.value)
			} else {
				ekmFunc_setSelectedIndex(document.getElementById('variant_' + item.option + '_' + item.value),item.value)
			}
		}
	}

	Variants_ProductsOptions();
}

function ekm_checkSelected(selectorObject) {
	returnValue = true;

	for (var i=0;i<selectorObject.length;i++){
		var item = selectorObject[i];

		if (item.hasOwnProperty('option') && item.hasOwnProperty('value')) {
			
			if (item.type == 'DROPDOWN') {
				if (document.getElementById('variant_' + item.option) && document.getElementById('variant_' + item.option).value.replace("'","\'") != item.value){
					returnValue = false;
				}
			} 
			else 
			{
				if (document.querySelector('input[name="' + CSS.escape(item.option) + '"]:checked') && document.querySelector('input[name="' + CSS.escape(item.option) + '"]:checked').value.replace("'","\'") != item.value){
					returnValue = false;
				} 
			} 
		}
	}

	return returnValue;
}

function ekmFunc_setSelectedIndex(s, v){
	if (s.options) {
		for (var i=0; i < s.options.length; i++ ){
			if 	(s.options[i].value==v) {
				s.options[i].selected=true;
				return;
			}
		}
	} else 	{
		s.checked = "checked";
		setRadioVariantData(s , s.name); //set the hidden fields value here (page_parser)
	}
}

function ekmFunc_addEvent(obj, evType, fn){
	if (obj.addEventListener){
		obj.addEventListener(evType, fn, false);
		return true;
	} else if (obj.attachEvent){
		var r=obj.attachEvent("on"+evType, fn);
		return r;
	}
	else{
		return false;
	}
}


function ekm_getElementsById(id){
	var loopLimit = 20;
	var element = document.getElementById(id);

	if (element === null) return [];

	var elements = [];
	while(loopLimit-- > 0){
		elements.push(element);
		element.id = 'tempId';
		element = document.getElementById(id);
		if (!element) break;
		if (element.id != id) break;
	}

	for(var i = 0; i < elements.length; i++){
		elements[i].id = id;
	}
	return elements;
}

function ekm_clearHiddenProductOptions() {
	var formOptions = document.querySelectorAll('.ekm-productoptions-dropdown-option option');
	for(var i = 0; i < formOptions.length; i++){
		if (formOptions[i].style) {
			formOptions[i].style.display='';
		}
	}
}


function ekm_IE_showhideOptions() {
	if ($ && (window.navigator.userAgent.indexOf("MSIE ")!=-1 || (!!navigator.userAgent.match(/Trident.*rv\:11\./)))) {
		$('.ekm-productoptions-dropdown-option').children().each(function() {
			var isSpan = $(this).is('span');
			var option = $(this);
			if(isSpan) {
				option = $($(this).children()[0]);
			}
			var isHidden = $(option).css("display") == 'none';
			if (isHidden) {
				if (!option.parent().is('span')) {
					option.wrap('<span style="display:none;"></span>');
				}
			} else {
				if (option.parent().is('span')) {
					option.parent().replaceWith(option);
				}
			}
		});
	}
}

var selectorList = [];

function buildAvaliableOptions() {
	selectorList = [];
	var selectors = [];
	var selector = {};
	var isDefaultInStock = true;
	if (!ekmHideOutOfStockVariantDataId) ekmHideOutOfStockVariantDataId = 0;
	for(var i = 0; i < ekmProductVariantData[ekmHideOutOfStockVariantDataId].items.length; i++) {
		var variant = ekmProductVariantData[ekmHideOutOfStockVariantDataId].items[i];
		var itemInStock = isDefaultInStock;
		for(var item in variant.properties) {
			var variantProperty = variant.properties[item];
			if (variantProperty.hasOwnProperty('elementId')){
				if(variantProperty.elementId == "_EKM_INSTOCK_VALUE") {
					if (variantProperty.value === 'True') {
						itemInStock = true;
						if (i==0) isDefaultInStock = true;
					} else {
						itemInStock = false;
						if (i==0) isDefaultInStock = false;
					}
				} 
			}
		}
		if(itemInStock) {
			selectorList.push({ "options" :variant.selector, "inStock" : "True"});	
		} else {
			selectorList.push({ "options" :variant.selector, "inStock" : "False"});
		}
	}
}

function itemInStock(item) {
	if(item.inStock == "True") return true;
	return false;
}

function itemFound(item1, item2) {
	if (item1.value == item2.value) return true;
	return false;
}

function itemAvailable(item1, item2, item3) {
	if(itemFound(item1, item2) && itemInStock(item3)) return true;
	return false;
}

function displaySelectors(event) {

	var currentId
	var noStock = false;
	var selectDropdowns = [];
	var selectElements = document.getElementsByTagName("select");
	var radioDropdowns = [];
	var radioElements = document.getElementsByTagName("input");

	var hasHitCurrent = false;
	var resetOption = false;
	var lastValidIndex = 0;
	var hasAvaliableOptions = false;

	for(var i = 0; i < selectElements.length; i++) {
		var element = selectElements[i];
		if(element.id.indexOf("variant_") > -1) {
			selectDropdowns.push(selectElements[i]);
		}
	}

	for(var i = 0; i < radioElements.length; i++) {
		var element = radioElements[i];
		if(element.id.indexOf("variant_") > -1  && element.type == "radio") {
			radioDropdowns.push(radioElements[i]);
		}
	}

	if (selectDropdowns.length != 0) {
		if (event == undefined) {
			currentId = selectDropdowns[0].id;
		} else {
			currentId = event.currentTarget.id;
		}

		for(var i = 0; i < selectDropdowns.length; i++) {
			var currentSelectDropdown = selectDropdowns[i];
			if(currentId === currentSelectDropdown.id) {
				hasHitCurrent = true;
			}

			if(hasHitCurrent) {
				for(var j = 0; j < currentSelectDropdown.options.length;j++) {
					noStock = false;

					for(var k = 0; k < selectorList.length; k++) {

						for(var l = 0; l < selectorList[k].options.length; l++) {

							if(itemAvailable(selectorList[k].options[l], currentSelectDropdown.options[j], selectorList[k])) {
								noStock = true;
								currentSelectDropdown.options[j].style.display = '';
								currentSelectDropdown.options[j].style.color='';
								currentSelectDropdown.options[j].disabled=false;
								
								lastValidIndex = j;
								
								hasAvaliableOptions = true;
								break;
							}
						}
					} 

					if (!noStock){

						currentSelectDropdown.options[j].style.display = 'none';
						currentSelectDropdown.options[j].style.color='#FFFFFF';
						currentSelectDropdown.options[j].disabled=true;

						if(currentSelectDropdown.selectedIndex == j) {
							resetOption = true;
						}
					}

					if(resetOption) {
						if(lastValidIndex == undefined) lastValidIndex = j;
						currentSelectDropdown.options[lastValidIndex].selected = "selected";
					}
				}
			}
			lastValidIndex = 0;
		}
	}

	for(var i = 0; i < radioDropdowns.length; i++) {

		var el = document.querySelector('label[for="'+radioDropdowns[i].id+'"]');

		noStock = false;

		for(var k = 0; k < selectorList.length; k++) {

			for(var l = 0; l < selectorList[k].options.length; l++) {

				if(itemAvailable(selectorList[k].options[l], radioDropdowns[i], selectorList[k])) {
					noStock = true;
					radioDropdowns[i].disabled=false;
					var el = document.querySelector('label[for="' + radioDropdowns[i].id + '"]')

					if(el){
						el.classList.remove("label-outofstock");
						el.style.display = '';
						el.style.opacity = '';
					} 		

					lastValidIndex = j;

					hasAvaliableOptions = true;
					break;
				}
			}
		} 

		if (!noStock){

			radioDropdowns[i].disabled=true;
			radioDropdowns[i].checked = "";

			var el = document.querySelector('label[for="' + radioDropdowns[i].id + '"]')

			if(el){
				el.style.setProperty ("display", "none", "important");
			}

			if(radioDropdowns[i].checked == "checked") {
				resetOption = true;
			}
		}

		if(resetOption) {
			if(lastValidIndex == undefined) lastValidIndex = j;
			radioDropdowns[lastValidIndex].checked = "checked";
		}

	}


	if(!hasAvaliableOptions) {

		for(var i = 0; i < selectDropdowns.length; i++) {
			selectDropdowns[i].disabled='disabled';
			selectDropdowns[i].style.opacity = '0.4';
		}

		for(var i = 0; i < radioDropdowns.length; i++) {
			radioDropdowns[i].disabled='disabled';
			radioDropdowns[i].checked = "";

			var el = document.querySelector('label[for="'+radioDropdowns[i].id+'"]');

			if(el){
				el.classList.add("label-outofstock");
				el.style.display = '';
				el.style.opacity = 0.4;
			}
		}
	} 

	//put the other variants of the current selection out of stock if required.
	var values = [];
	var names = [];

	//get all the selects name and values
	for (var i=0; i<selectDropdowns.length; i++){
		values[values.length] = {
			name: selectDropdowns[i].name,
			value: selectDropdowns[i].value,
			kind: 'dropdown'
		}
	}

	//get all the colour name and values
	for (var i = 0; i < radioDropdowns.length; i++){
		if (radioDropdowns[i].checked){
			values[values.length] = {
				name: 'variant_' + radioDropdowns[i].name,
				value: radioDropdowns[i].value,
				kind: 'radio'
			}
		}
	}

	for (var i = 0; i < radioDropdowns.length; i++) {

		//remove any radio options which have the same name as the current loop target
		var localValues = values.filter(function(value){
			return value.name !== 'variant_' + radioDropdowns[i].name;
		});


		//an array of combinations that contain the current selections
		var combinations = selectorList.filter(function(v){
			return v.options.some(function(o){
				return (o.type === 'COLOUR' || o.type === 'SIZE') && o.option === radioDropdowns[i].name && o.value === radioDropdowns[i].value;
			});
		});

		var outOfStock = combinations.some(function(v){
			if (v.inStock == 'True') return false;

			//all the variants which aren't the current selection
			var filteredVariants = v.options.filter(function(o){ 
				return o.option !== radioDropdowns[i].name;
			});

			var matches = 0;

			//loop through all the variants options and see if the combination contains the current selection
			filteredVariants.forEach(function(o){
				for (var j=0; j<localValues.length; j++){
					if (localValues[j].name === 'variant_' + o.option && localValues[j].value === o.value){
						matches++;
					}
				}
			});


			//number of variants that have to be matched in order to pass the lower check
			var targetValues = localValues.reduce(function(a, value){
				if (value.kind == 'dropdown'){
					a.push(value);
					return a;
				}

				//not already checked
				var isDup = a.some(function(v2){
					return v2.name === value.name;
				});

				//if not a dup add it
				if (!isDup) a.push(value);
					return a;
			}, []);

				var target = targetValues.length;
				return matches == target;
		});

			//if out of stock push to array
		if (outOfStock) names[names.length] = {
			name: radioDropdowns[i].name,
			value: radioDropdowns[i].value,
			element: radioDropdowns[i]
		};
	}
		
		//for all the outofstock variants add the outofstock class and styles
	names.forEach(function (t){
		var id = t.element.id;
		var labels = document.querySelectorAll('label[for="' + id + '"]');
		[].forEach.call(labels, function(label){
			label.classList.add('label-outofstock');
			label.style.opacity = 0.4;
		});
	});

}
