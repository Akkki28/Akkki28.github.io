/*
	Miniport by HTML5 UP
	html5up.net | @ajlkn
	Free for personal and commercial use under the CCA 3.0 license (html5up.net/license)
*/

(function($) {

	var	$window = $(window),
		$body = $('body'),
		$nav = $('#nav');

	// Breakpoints.
		breakpoints({
			xlarge:  [ '1281px',  '1680px' ],
			large:   [ '981px',   '1280px' ],
			medium:  [ '737px',   '980px'  ],
			small:   [ null,      '736px'  ]
		});

	// Play initial animations on page load.
		$window.on('load', function() {
			window.setTimeout(function() {
				$body.removeClass('is-preload');
			}, 100);
		});

	// Scrolly.
		$('#nav a, .scrolly').scrolly({
			speed: 1000,
			offset: function() { return $nav.height(); }
		});

})(jQuery);

// Project filtering system
(function($) {
    $(document).ready(function() {
        // Project filtering
        $('.filter-tags a').on('click', function(e) {
            e.preventDefault();
            
            // Remove active class from all filter tags
            $('.filter-tags a').removeClass('active');
            
            // Add active class to clicked tag
            $(this).addClass('active');
            
            // Get the filter value
            const filter = $(this).attr('data-filter');
            
            // Show or hide projects based on filter
            if (filter === 'all') {
                $('[data-tags]').removeClass('hidden');
            } else {
                $('[data-tags]').each(function() {
                    const tags = $(this).attr('data-tags').split(' ');
                    if (tags.includes(filter)) {
                        $(this).removeClass('hidden');
                    } else {
                        $(this).addClass('hidden');
                    }
                });
            }
        });
    });
})(jQuery);