from __future__ import print_function
import boost_apriltags as apriltags

d = apriltags.TagDetector()
d.test()
# print apriltags.test_fn()
d.setup()

while True:
    d.detect_apriltags()
    if d.num_detected() > 0:
        print (d.num_detected())
        break

for x in range(d.num_detected()):
    print("Tag: ", d.getTag(x).id)
