try:
    from datetime import datetime

    from debugging import RobotLogger, System
    from end_of_play import end_of_play_routine
    from maze import Maze
    from setup import setup_delta_coordinate_mapper, setup_robot
    from solve_map import solve_map
    from types_and_constants import DEBUG, EndOfTimeError, LackOfProgressError

    logger = RobotLogger()
    logger.info(f"Começando nova execução: {datetime.now()}", System.initialization)

    robot = setup_robot(logger)
    maze = Maze(logger)

    try:
        setup_delta_coordinate_mapper(robot, maze)
        solve_map(robot, logger, maze)
    except EndOfTimeError:
        pass
    except LackOfProgressError:
        pass
    except Exception:
        if DEBUG:
            logger.critical(
                "Erro inesperado enquanto resolvia o mapa",
                System.unknown_error,
            )
            logger.flush()
            raise

    end_of_play_routine(robot, maze, logger)

    logger.flush()
except Exception:
    try:  # ? Ensures robot is initialized in order to guarantee that `print` will be shown
        import os

        from controller import Robot as WebotsRobot  # type: ignore

        webots_robot = WebotsRobot()
        webots_robot.step(int(os.getenv("TIME_STEP", 32)))
    except Exception:
        ...

    import sys
    import traceback

    exc_info = sys.exc_info()

    print(
        f"Unknown general exception: {''.join(traceback.format_exception(*exc_info))}"
    )
