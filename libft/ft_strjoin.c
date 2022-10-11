/* ************************************************************************** */
/*                                                                            */
/*                                                        :::      ::::::::   */
/*   ft_strjoin.c                                       :+:      :+:    :+:   */
/*                                                    +:+ +:+         +:+     */
/*   By: youskim <youskim@student.42seoul.kr>       +#+  +:+       +#+        */
/*                                                +#+#+#+#+#+   +#+           */
/*   Created: 2021/11/17 16:13:44 by youskim           #+#    #+#             */
/*   Updated: 2022/05/17 18:40:11 by youskim          ###   ########.fr       */
/*                                                                            */
/* ************************************************************************** */

#include <stdlib.h>
#include "libft.h"

static char	*ft_strcpy(char const *s1, char *arr, int start, int len)
{
	int	i;

	i = 0;
	while (i < len)
	{
		arr[start] = s1[i];
		start++;
		i++;
	}
	return (arr);
}

char	*ft_strjoin(char const *s1, char const *s2)
{
	int		len_s1;
	int		len_s2;
	char	*arr;

	len_s1 = ft_strlen(s1);
	len_s2 = ft_strlen(s2);
	arr = (char *)malloc(sizeof(char) * (len_s1 + len_s2 + 1));
	if (arr == 0)
		return (NULL);
	ft_strcpy(s1, arr, 0, len_s1);
	ft_strcpy(s2, arr, len_s1, len_s2);
	arr[len_s1 + len_s2] = '\0';
	return (arr);
}
