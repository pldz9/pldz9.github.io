require(['gitbook', 'jQuery'], function (gitbook, $) {
    var PLUGIN = 'expandable-chapter-small2',
        TOGGLE_CLASSNAME = 'expanded',
        CHAPTER = '.chapter',
        // ARTICLES = '.articles',
        ARTICLES = '.chapter ul',
        FOLDABLE = '.chapter, .chapter li',
        ARTICLE_CHILDREN = 'ul',
        TRIGGER_TEMPLATE = '<i class="exc-trigger fa"></i>',
        EXPAND_CATEGORY_STORE = [];

    var init = function () {
        // adding the trigger element to each ARTICLES parent and binding the event
        $(ARTICLES)
        .parent(CHAPTER)
        .find(ARTICLE_CHILDREN)
        .prev()
        .css('cursor', 'pointer')
        .on('click', function (e) {
            e.preventDefault();
            e.stopPropagation();
            toggle($(e.target).closest(FOLDABLE));
        })
        .append(TRIGGER_TEMPLATE);

        // ⭐ Not expand all item.
        // expand(lsItem());

        // expand current selected chapter with it's parents
        var activeChapter = $(CHAPTER + '.active');

        // ⭐ Not need to expand active chapter itself.
        // expand(activeChapter);

        // expand current selected chapter's children
        expand(activeChapter.parents(CHAPTER));

        showExpandedChapters();
    }

    var toggle = function ($chapter) {
        if ($chapter.hasClass('expanded')) {
            collapse($chapter);
        } else {
            expand($chapter);
        }
    }

    var collapse = function ($chapter) {
        if ($chapter.length && $chapter.hasClass(TOGGLE_CLASSNAME)) {
            $chapter.removeClass(TOGGLE_CLASSNAME);
            
            const id = $chapter[0].id;
            const elemIndex = EXPAND_CATEGORY_STORE.findIndex(element => element === id);
            if (elemIndex !== -1) {
                EXPAND_CATEGORY_STORE.splice(elemIndex, 1);
            }
        }
    }

    var expand = function ($chapter) {
        if ($chapter.length && !$chapter.hasClass(TOGGLE_CLASSNAME)) {
            $chapter.addClass(TOGGLE_CLASSNAME);

            const currentId = $chapter[0].id;
            const elem = EXPAND_CATEGORY_STORE.find(element => element === currentId);
            if(!elem){
                EXPAND_CATEGORY_STORE.push(currentId);
            }
        }
    }

    var lsItem = function () {
        //
    }

    var showExpandedChapters = function () {
        EXPAND_CATEGORY_STORE.forEach((id) => {
            const categoryElem = $(`#${id}`)[0]
            if(!categoryElem.classList.contains(TOGGLE_CLASSNAME)){
                categoryElem.classList.add(TOGGLE_CLASSNAME)
            }
        })

        requestAnimationFrame(() => {
            const target = document.querySelector('#book-summary-core .chapter.active');
            if (target) {
                target.scrollIntoView({
                    behavior: 'auto',
                    block: 'center'
                });
            }
        });
    }

    gitbook.events.bind('page.change', function () {
        init();
    });
});
