# Faults are deliberate test samples; the camera still decides their outcome.
import random


def make_faults(total, count=0, placement="random", indices="", seed=0):
    if type(count) is not int or not 0 <= count <= total:
        raise ValueError("Faulty products must be between 0 and the product count")
    if placement == "random":
        positions = sorted(random.Random(seed).sample(range(1, total + 1), count))
    elif placement == "first":
        positions = list(range(1, count + 1))
    elif placement == "last":
        positions = list(range(total - count + 1, total + 1))
    elif placement == "custom":
        try:
            positions = sorted(
                int(value.strip()) for value in indices.split(",") if value.strip()
            )
        except (TypeError, ValueError):
            raise ValueError(
                "Use product numbers separated by commas, such as 2,4"
            ) from None
        if (
            len(positions) != count
            or len(set(positions)) != count
            or any(not 1 <= value <= total for value in positions)
        ):
            raise ValueError("Choose exactly that many different product numbers")
    else:
        raise ValueError("Unknown faulty-product placement")
    kinds = ("missing_gear", "unseated_insert")
    return {toy: kinds[index % 2] for index, toy in enumerate(positions)}
