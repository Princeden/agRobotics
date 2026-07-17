Two Projects

- Visual Servoing
- Cow Project

### Visual Servoing

- [X] rewrite the code so that marker / aruco data is shared between relevant nodes
- [ ] write a servo orchestrator that switches between
- [ ] test in sim?
- [ ] create retry logic

How would the orchestrator work?

- have movegroup, endeffector, information about the april tag
- want to move it to position
- also have the problem of constraints, wrist 2 and wrist 3.
- move cartesian path as much as possible
- once it fails that switch to servoing
- can use a set time?
- utilize an fsm

### Cow Project

Todos

- [ ] need to figure outfeatures / a high level overview
