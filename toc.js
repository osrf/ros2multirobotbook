// Populate the sidebar
//
// This is a script, and not included directly in the page, to control the total size of the book.
// The TOC contains an entry for each page, so if each page includes a copy of the TOC,
// the total size of the page becomes O(n**2).
class MDBookSidebarScrollbox extends HTMLElement {
    constructor() {
        super();
    }
    connectedCallback() {
        this.innerHTML = '<ol class="chapter"><li class="chapter-item "><a href="intro.html"><strong aria-hidden="true">1.</strong> Introduction</a><a class="toggle"><div>❱</div></a></li><li><ol class="section"><li class="chapter-item "><a href="installation.html"><strong aria-hidden="true">1.1.</strong> Installation</a></li><li class="chapter-item "><a href="demos.html"><strong aria-hidden="true">1.2.</strong> Demos</a></li></ol></li><li class="chapter-item "><a href="ros2.html"><strong aria-hidden="true">2.</strong> ROS 2</a><a class="toggle"><div>❱</div></a></li><li><ol class="section"><li class="chapter-item "><a href="ros2_tools_resources.html"><strong aria-hidden="true">2.1.</strong> ROS Resources</a></li><li class="chapter-item "><a href="ros2_design_patterns.html"><strong aria-hidden="true">2.2.</strong> ROS Concepts and Design Patterns</a></li><li class="chapter-item "><a href="ros2_cli.html"><strong aria-hidden="true">2.3.</strong> The ROS Command Line Interface</a></li><li class="chapter-item "><a href="ros2_api.html"><strong aria-hidden="true">2.4.</strong> The ROS API</a></li></ol></li><li class="chapter-item "><a href="traffic-editor.html"><strong aria-hidden="true">3.</strong> Traffic Editor</a></li><li class="chapter-item "><a href="simulation.html"><strong aria-hidden="true">4.</strong> Simulation</a></li><li class="chapter-item "><a href="rmf-core.html"><strong aria-hidden="true">5.</strong> RMF Core Overview</a><a class="toggle"><div>❱</div></a></li><li><ol class="section"><li class="chapter-item "><a href="rmf-core_faq.html"><strong aria-hidden="true">5.1.</strong> Frequently Asked Questions</a></li></ol></li><li class="chapter-item "><a href="task.html"><strong aria-hidden="true">6.</strong> Tasks in RMF</a><a class="toggle"><div>❱</div></a></li><li><ol class="section"><li class="chapter-item "><a href="task_planner.html"><strong aria-hidden="true">6.1.</strong> RMF Task Allocation Planner</a></li><li class="chapter-item "><a href="task_types.html"><strong aria-hidden="true">6.2.</strong> Currently supported Tasks</a></li><li class="chapter-item "><a href="task_userdefined.html"><strong aria-hidden="true">6.3.</strong> User-defined Tasks</a></li><li class="chapter-item "><a href="task_new.html"><strong aria-hidden="true">6.4.</strong> Supporting a new Task</a></li></ol></li><li class="chapter-item "><a href="soss.html"><strong aria-hidden="true">7.</strong> SOSS</a></li><li class="chapter-item "><a href="integration.html"><strong aria-hidden="true">8.</strong> Integration</a><a class="toggle"><div>❱</div></a></li><li><ol class="section"><li class="chapter-item "><a href="integration_nav-maps.html"><strong aria-hidden="true">8.1.</strong> Navigation Maps</a></li><li class="chapter-item "><a href="integration_fleets.html"><strong aria-hidden="true">8.2.</strong> Mobile Robot Fleets</a><a class="toggle"><div>❱</div></a></li><li><ol class="section"><li class="chapter-item "><a href="integration_fleets_adapter_tutorial.html"><strong aria-hidden="true">8.2.1.</strong> Fleet Adapter Tutorial</a></li><li class="chapter-item "><a href="integration_fleets_action_tutorial.html"><strong aria-hidden="true">8.2.2.</strong> PerformAction Tutorial</a></li><li class="chapter-item "><a href="integration_free_fleet_adapter.html"><strong aria-hidden="true">8.2.3.</strong> Free Fleet Adapter</a><a class="toggle"><div>❱</div></a></li><li><ol class="section"><li class="chapter-item "><a href="integration_free-fleet.html"><strong aria-hidden="true">8.2.3.1.</strong> Legacy - free fleet</a></li></ol></li></ol></li><li class="chapter-item "><a href="integration_read-only.html"><strong aria-hidden="true">8.3.</strong> Read-Only Fleets</a></li><li class="chapter-item "><a href="integration_doors.html"><strong aria-hidden="true">8.4.</strong> Doors</a></li><li class="chapter-item "><a href="integration_lifts.html"><strong aria-hidden="true">8.5.</strong> Lifts (Elevators)</a></li><li class="chapter-item "><a href="integration_workcells.html"><strong aria-hidden="true">8.6.</strong> Workcells</a></li></ol></li><li class="chapter-item "><a href="rmf-web.html"><strong aria-hidden="true">9.</strong> RMF Web</a><a class="toggle"><div>❱</div></a></li><li><ol class="section"><li class="chapter-item "><a href="ui.html"><strong aria-hidden="true">9.1.</strong> User Interfaces</a></li></ol></li><li class="chapter-item "><a href="security.html"><strong aria-hidden="true">10.</strong> Security</a></li><li class="chapter-item "><a href="roadmap.html"><strong aria-hidden="true">11.</strong> Roadmap</a></li><li class="chapter-item "><a href="howto_guides.html"><strong aria-hidden="true">12.</strong> How-to Guides</a><a class="toggle"><div>❱</div></a></li><li><ol class="section"><li class="chapter-item "><a href="integration_nav-maps-strategies.html"><strong aria-hidden="true">12.1.</strong> Graph Strategies</a></li></ol></li></ol>';
        // Set the current, active page, and reveal it if it's hidden
        let current_page = document.location.href.toString().split("#")[0].split("?")[0];
        if (current_page.endsWith("/")) {
            current_page += "index.html";
        }
        var links = Array.prototype.slice.call(this.querySelectorAll("a"));
        var l = links.length;
        for (var i = 0; i < l; ++i) {
            var link = links[i];
            var href = link.getAttribute("href");
            if (href && !href.startsWith("#") && !/^(?:[a-z+]+:)?\/\//.test(href)) {
                link.href = path_to_root + href;
            }
            // The "index" page is supposed to alias the first chapter in the book.
            if (link.href === current_page || (i === 0 && path_to_root === "" && current_page.endsWith("/index.html"))) {
                link.classList.add("active");
                var parent = link.parentElement;
                if (parent && parent.classList.contains("chapter-item")) {
                    parent.classList.add("expanded");
                }
                while (parent) {
                    if (parent.tagName === "LI" && parent.previousElementSibling) {
                        if (parent.previousElementSibling.classList.contains("chapter-item")) {
                            parent.previousElementSibling.classList.add("expanded");
                        }
                    }
                    parent = parent.parentElement;
                }
            }
        }
        // Track and set sidebar scroll position
        this.addEventListener('click', function(e) {
            if (e.target.tagName === 'A') {
                sessionStorage.setItem('sidebar-scroll', this.scrollTop);
            }
        }, { passive: true });
        var sidebarScrollTop = sessionStorage.getItem('sidebar-scroll');
        sessionStorage.removeItem('sidebar-scroll');
        if (sidebarScrollTop) {
            // preserve sidebar scroll position when navigating via links within sidebar
            this.scrollTop = sidebarScrollTop;
        } else {
            // scroll sidebar to current active section when navigating via "next/previous chapter" buttons
            var activeSection = document.querySelector('#sidebar .active');
            if (activeSection) {
                activeSection.scrollIntoView({ block: 'center' });
            }
        }
        // Toggle buttons
        var sidebarAnchorToggles = document.querySelectorAll('#sidebar a.toggle');
        function toggleSection(ev) {
            ev.currentTarget.parentElement.classList.toggle('expanded');
        }
        Array.from(sidebarAnchorToggles).forEach(function (el) {
            el.addEventListener('click', toggleSection);
        });
    }
}
window.customElements.define("mdbook-sidebar-scrollbox", MDBookSidebarScrollbox);
