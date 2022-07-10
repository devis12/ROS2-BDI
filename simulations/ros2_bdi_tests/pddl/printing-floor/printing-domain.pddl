(define (domain printing-domain)

    (:requirements :strips :typing :fluents :durative-actions)

    ;; Types ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
    (:types
        room hallway_segment - area
        dock
        robot
        printer
    );; end Types ;;;;;;;;;;;;;;;;;;;;;;;;;

    (:predicates
        (near ?a1 ?a2 - area)
        (r_in ?r - robot ?a - area)
        (r_docked ?r - robot)
        (not_r_docked ?r - robot)
        (d_in ?d - dock ?r - room)
        (p_in ?p - printer ?h - hallway_segment)
        (free ?a - area)
        (inactive ?r - robot)
        (active ?r - robot)
        (available ?p - printer)
        (printed_docs_loaded ?r - robot)
        (printed_docs_left_in ?r - robot ?a - area)
        (fully_recharged ?r - robot)
    )

    (:functions
        (battery_charge ?r - robot)
    )

    (:durative-action move
        :parameters (?r - robot ?a1 ?a2 - area)
        :duration (= ?duration 4)
        :condition (and
            (at start (r_in ?r ?a1))
            (over all (active ?r))
            (over all (not_r_docked ?r))
            (over all (free ?a2))
            (over all (near ?a1 ?a2))
        )
        :effect (and
            (at start (not(r_in ?r ?a1)))
            (at end (r_in ?r ?a2))
        )
    )

    (:durative-action printing
        :parameters (?r - robot ?p - printer ?hs - hallway_segment)
        :duration (= ?duration 4)
        :condition (and
            (at start (available ?p))
            (over all (active ?r))
            (over all (not_r_docked ?r))
            (over all (r_in ?r ?hs))
            (over all (p_in ?p ?hs))
        )
        :effect (and
            (at start (not(available ?p)))
            (at end (printed_docs_loaded ?r))
            (at end (available ?p))
        )
    )

    (:durative-action unload_printed_docs
        :parameters (?r - robot ?a - area)
        :duration (= ?duration 2)
        :condition (and
            (at start (printed_docs_loaded ?r))
            (over all (not_r_docked ?r))
            (over all (active ?r))
            (over all (r_in ?r ?a))
        )
        :effect (and
            (at end (not(printed_docs_loaded ?r)))
            (at end (printed_docs_left_in ?r ?a))
        )
    )

    (:durative-action recharge
        :parameters (?r - robot ?d - dock ?room - room)
        :duration (= ?duration 2)
        :condition (and
            (over all (d_in ?d ?room))
            (over all (r_docked ?r))
            (over all (inactive ?r))
            (over all (r_in ?r ?room))
        )
        :effect (and
            (at end (fully_recharged ?r))
        )
    )

    (:durative-action docking
        :parameters (?r - robot ?d - dock ?room - room)
        :duration (= ?duration 1)
        :condition (and
            (at start (not_r_docked ?r))
            (over all (active ?r))
            (over all (d_in ?d ?room))
            (over all (r_in ?r ?room))
        )
        :effect (and
            (at end (not(not_r_docked ?r)))
            (at end (r_docked ?r))
        )
    )

    (:durative-action undocking
        :parameters (?r - robot ?d - dock ?room - room)
        :duration (= ?duration 1)
        :condition (and
            (at start (r_docked ?r))
            (over all (active ?r))
            (over all (d_in ?d ?room))
            (over all (r_in ?r ?room))
        )
        :effect (and
            (at end (not_r_docked ?r))
            (at end (not(r_docked ?r)))
        )
    )
)
