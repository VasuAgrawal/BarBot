import math

def almostEquals(a, b):
    return abs(a - b) <= 0.000001

def convert(heading):
    heading += 3 * 2 * math.pi
    heading %= 2*math.pi
    if heading > math.pi:
        heading -= 2*math.pi
    return heading

def testConvert():
    print("Testing conversion...")
    assert(almostEquals(convert(math.radians(60)), math.radians(60)))
    assert(almostEquals(convert(math.radians(120)), math.radians(120)))
    assert(almostEquals(convert(math.radians(180)), math.radians(180)))
    assert(almostEquals(convert(math.radians(190)), math.radians(-170)))
    assert(almostEquals(convert(math.radians(200)), math.radians(-160)))
    print("Passed!")

testConvert()