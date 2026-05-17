function R = projectToSO3(A)
%PROJECTTOSO3 Проецирует матрицу на SO(3) после численного интегрирования.

    [U, ~, V] = svd(A);
    R = U * V';

    if det(R) < 0
        U(:, 3) = -U(:, 3);
        R = U * V';
    end
end
