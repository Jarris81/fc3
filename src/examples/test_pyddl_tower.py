from pyddl import Domain, Problem, Action, neg, planner

def problem(verbose):

    domain = Domain((
        Action("pick-up",
               parameters=(
                   ("block", "b"),
                   ("gripper", "g")
               ),
               preconditions=(
                   ("free", "b"),
                   ("empty", "g")

               ),
               effects=(
                   ("inhand", "g", "b"),
                   neg(("free", "b")),
                   neg(("empty", "g"))
               )),
        Action("place_on",
               parameters=(
                   ("block", "b"),
                   ("gripper", "g"),
                   ("block", "c")
               ),
               preconditions=(
                   ("inhand","g", "b"),
                   ("free", "c")
               ),
               effects=(
                   neg(("inhand", "g", "b")),
                   neg(("free", "c")),
                   ("free", "b"),
                   ("empty", "g"),
                   ("on", "b", "c")

               ))
    ))
    prob = Problem(
        domain,
        {
            'block': (1, 2, 3),
            "gripper": "g"
        },
        init=(
            ("empty", "g"),
            ("free", 1),
            ("free", 2),
            ("free", 3)
        ),
        goal=(
            ("on", 1, 2),
            ("on", 2, 3),
            ("empty", "g")
        )
    )
    plan = planner(prob, verbose=verbose)
    if plan is None:
        print('No Plan!')
    else:
        for action in plan:
            print(action)

if __name__ == '__main__':
    from optparse import OptionParser

    parser = OptionParser(usage="Usage: %prog [options]")
    parser.add_option('-q', '--quiet',
                      action='store_false', dest='verbose', default=True,
                      help="don't print statistics to stdout")

    # Parse arguments
    opts, args = parser.parse_args()
    problem(opts.verbose)