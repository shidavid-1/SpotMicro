///// Guided Hardware Test Session /////
/* An interactive walkthrough of the manual tests defined in the TPUP test
   plan. Runs over the serial monitor: pick a test from the menu, follow the
   printed instructions, and record PASS/FAIL/SKIP when prompted. A summary
   table prints on exit.

   This covers the tests that need a human in the loop. The pure math checks
   live in Self_Test.hpp and run without any prompting.

   USAGE: idf.py flash monitor, then type a menu number and press Enter.
*/

#ifndef TEST_SESSION_HPP
#define TEST_SESSION_HPP

// REQUIRES: init_i2c() has already been called.
// EFFECTS: Runs the interactive menu until the user quits, then prints a
// summary of every test's recorded result.
void run_test_session();

#endif // TEST_SESSION_HPP
