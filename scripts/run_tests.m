%RUN_TESTS Запускает все тесты проекта и выводит таблицу результатов.

rootDir = fileparts(fileparts(mfilename('fullpath')));
testResults = runtests(fullfile(rootDir, 'tests'));

disp(table(testResults));
assertSuccess(testResults);
